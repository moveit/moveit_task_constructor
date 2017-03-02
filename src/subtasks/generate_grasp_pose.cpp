#include <moveit_task_constructor/subtasks/generate_grasp_pose.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

#include <chrono>
#include <functional>

moveit::task_constructor::subtasks::GenerateGraspPose::GenerateGraspPose(std::string name)
: moveit::task_constructor::SubTask::SubTask(name),
  timeout_(0.1),
  angle_delta_(0.1),
  current_angle_(0.0),
  remaining_time_(timeout_),
  tried_current_state_as_seed_(false)
{
	ros::NodeHandle nh;
	pub= nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 50);
	ros::Duration(1.0).sleep();
}

void
moveit::task_constructor::subtasks::GenerateGraspPose::setGroup(std::string group){
	group_= group;
}

void
moveit::task_constructor::subtasks::GenerateGraspPose::setEndEffector(std::string eef){
	eef_= eef;
}

void
moveit::task_constructor::subtasks::GenerateGraspPose::setObject(std::string object){
	object_= object;
}

void
moveit::task_constructor::subtasks::GenerateGraspPose::setTimeout(double timeout){
	timeout_= timeout;
	remaining_time_= timeout;
}

void
moveit::task_constructor::subtasks::GenerateGraspPose::setAngleDelta(double delta){
	angle_delta_= delta;
}


bool
moveit::task_constructor::subtasks::GenerateGraspPose::canCompute(){
	return current_angle_ < 2*M_PI;
}

namespace {
	bool isValid(planning_scene::PlanningSceneConstPtr scene,
	        std::vector< std::vector<double> >& old_solutions,
	        robot_state::RobotState* state,
	        const robot_model::JointModelGroup* jmg,
	        const double* joint_positions){
		for( std::vector<double> sol : old_solutions ){
			if( jmg->distance(joint_positions, sol.data()) < 0.1 ){
				return false;
			}
		}
		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		return scene->isStateColliding(*state, jmg->getName());
	}
}

bool
moveit::task_constructor::subtasks::GenerateGraspPose::compute(){
	planning_scene::PlanningScenePtr grasp_scene = scene_->diff();
	robot_state::RobotState &grasp_state = grasp_scene->getCurrentStateNonConst();

	// empty trajectory -> this subtask only produces states
	const robot_trajectory::RobotTrajectoryPtr traj;

	const moveit::core::JointModelGroup* jmg= grasp_state.getJointModelGroup(group_);

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			scene_,
			previous_solutions_,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);


	geometry_msgs::Pose pose;
	const Eigen::Affine3d object_pose= scene_->getFrameTransform(object_);
	if(object_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	tf::poseEigenToMsg(object_pose, pose);

	while(current_angle_ < 2*M_PI){
		if( remaining_time_ <= 0.0 ){
			current_angle_+= angle_delta_;
			remaining_time_= timeout_;
			tried_current_state_as_seed_= false;
			previous_solutions_.clear();
			continue;
		}

		pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, current_angle_);
		std::cout << "trying " << current_angle_ << " with remaining time " << remaining_time_ << std::endl;

		if(tried_current_state_as_seed_)
			grasp_state.setToRandomPositions(jmg);
		tried_current_state_as_seed_= true;

		auto now= std::chrono::steady_clock::now();
		bool succeeded= grasp_state.setFromIK(jmg, pose, eef_, 1, remaining_time_, is_valid);
		remaining_time_-= std::chrono::duration<double>(std::chrono::steady_clock::now()- now).count();

		if(succeeded){
			moveit_msgs::DisplayRobotState drs;
			robot_state::robotStateToRobotStateMsg(grasp_state, drs.state);
			pub.publish(drs);

			previous_solutions_.emplace_back();
			grasp_state.copyJointGroupPositions(jmg, previous_solutions_.back());
			moveit::task_constructor::SubTrajectory &trajectory = addTrajectory(traj);
			sendBothWays(trajectory, grasp_scene);
			return true;
		}
	}

	return false;
}
