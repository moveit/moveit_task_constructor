#include <moveit_task_constructor/subtasks/generate_grasp_pose.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

#include <chrono>

moveit::task_constructor::subtasks::GenerateGraspPose::GenerateGraspPose(std::string name)
: moveit::task_constructor::SubTask::SubTask(name),
  timeout_(0.1),
  current_angle_(0.0),
  remaining_time_(timeout_)
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


bool
moveit::task_constructor::subtasks::GenerateGraspPose::canCompute(){
	return current_angle_ < 2*M_PI;
}

bool
moveit::task_constructor::subtasks::GenerateGraspPose::compute(){
	planning_scene::PlanningScenePtr grasp_scene = scene_->diff();
	robot_state::RobotState &grasp_state = grasp_scene->getCurrentStateNonConst();

	// empty trajectory -> this subtask only produces states
	const robot_trajectory::RobotTrajectoryPtr traj;

	const moveit::core::JointModelGroup* jmg= grasp_state.getJointModelGroup(group_);

	geometry_msgs::Pose pose;
	const Eigen::Affine3d object_pose= scene_->getFrameTransform(object_);
	if(object_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	tf::poseEigenToMsg(object_pose, pose);

	for(; current_angle_ < 2*M_PI; current_angle_+= 0.2, remaining_time_= timeout_){
		if( remaining_time_ <= 0.0 )
			continue;

		pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, current_angle_);
		std::cout << "trying " << current_angle_ << std::endl;

		auto now= std::chrono::steady_clock::now();
		bool succeeded= grasp_state.setFromIK(jmg, pose, eef_, remaining_time_/*,validState*/);
		remaining_time_-= std::chrono::duration<double>(std::chrono::steady_clock::now()- now).count();

		moveit_msgs::DisplayRobotState drs;
		robot_state::robotStateToRobotStateMsg(grasp_state, drs.state);
		pub.publish(drs);

		if(succeeded){
			moveit::task_constructor::SubTrajectory &trajectory = addTrajectory(traj);
			sendBothWays(trajectory, grasp_scene);
			return true;
		}
	}

	return false;
}
