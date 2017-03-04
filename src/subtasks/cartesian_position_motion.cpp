#include <moveit_task_constructor/subtasks/cartesian_position_motion.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

moveit::task_constructor::subtasks::CartesianPositionMotion::CartesianPositionMotion(std::string name)
: moveit::task_constructor::SubTask::SubTask(name)
{
	ros::NodeHandle nh;
	pub= nh.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 50);
	ros::Duration(1.0).sleep();
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::setGroup(std::string group){
	group_= group;
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::setLink(std::string link){
	link_= link;
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::setMinDistance(double distance){
	min_distance_= distance;
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::setMaxDistance(double distance){
	max_distance_= distance;
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::setMinMaxDistance(double min_distance, double max_distance){
	setMinDistance(min_distance);
	setMaxDistance(max_distance);
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::towards(geometry_msgs::PointStamped towards){
	mode_= "towards";
	towards_= towards;
}

void
moveit::task_constructor::subtasks::CartesianPositionMotion::along(geometry_msgs::Vector3Stamped along){
	mode_= "along";
	along_= along;
}

bool
moveit::task_constructor::subtasks::CartesianPositionMotion::canCompute(){
	return hasEnding(); //hasBeginning() || hasEnding();
}

namespace {
	bool isValid(planning_scene::PlanningSceneConstPtr scene,
	        robot_state::RobotState* state,
	        const robot_model::JointModelGroup* jmg,
	        const double* joint_positions){
		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		return !scene->isStateColliding(*state, jmg->getName());
	}
}

bool
moveit::task_constructor::subtasks::CartesianPositionMotion::compute(){
	if( mode_ == "towards" && hasEnding() ){
		moveit::task_constructor::InterfaceState& ending= fetchStateEnding();

		planning_scene::PlanningScenePtr result_scene = ending.state->diff();
		robot_state::RobotState &robot_state = result_scene->getCurrentStateNonConst();

		const moveit::core::JointModelGroup* jmg= robot_state.getJointModelGroup(group_);
		const moveit::core::LinkModel* link_model= robot_state.getRobotModel()->getLinkModel(link_);

		auto traj= std::make_shared<robot_trajectory::RobotTrajectory>(robot_state.getRobotModel(), jmg);

		const moveit::core::GroupStateValidityCallbackFn is_valid=
			std::bind(
				&isValid,
				result_scene,
				std::placeholders::_1,
				std::placeholders::_2,
				std::placeholders::_3);

		const Eigen::Affine3d& link_pose= robot_state.getGlobalLinkTransform(link_);
		// should compute the direction away from the object along the current gripper direction
		const Eigen::Vector3d direction= link_pose.linear()*Eigen::Vector3d(-1,0,0);

		std::vector<moveit::core::RobotStatePtr> trajectory_steps;
		double achieved_fraction= robot_state.computeCartesianPath(
			jmg,
			trajectory_steps,
			link_model,
			direction,
			true, /* global frame */
			max_distance_, /* distance */
			.005, /* cartesian step size */
			1.5, /* jump threshold */
			is_valid);
		std::cout << "achieved " << achieved_fraction << " of cartesian motion" << std::endl;

		//TODO set succeeded here
		bool succeeded= achieved_fraction > 0.0;

		for( std::vector<moveit::core::RobotStatePtr>::iterator i= trajectory_steps.begin(); i != trajectory_steps.end(); ++i){
			traj->addPrefixWayPoint(*i, 0.0);
		}

		if(succeeded){
			moveit::core::RobotStatePtr result_state= trajectory_steps.back();

			moveit_msgs::DisplayTrajectory dt;
			robot_state::robotStateToRobotStateMsg(*result_state, dt.trajectory_start);
			dt.trajectory.resize(1);
			traj->getRobotTrajectoryMsg(dt.trajectory[0]);
			dt.model_id= "tams_ur5_setup";
			pub.publish(dt);

			robot_state= *result_state;

			moveit::task_constructor::SubTrajectory &trajectory = addTrajectory(traj);
			sendBackward(trajectory, result_scene);
			return true;
		}
	}
	else if( mode_ == "along" ){

	}
	else {
		throw std::runtime_error("position motion has neither a goal nor a direction");
	}
	return false;
}
