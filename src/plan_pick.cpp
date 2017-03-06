#include <moveit_task_constructor/task.h>

#include <moveit_task_constructor/subtasks/current_state.h>
#include <moveit_task_constructor/subtasks/gripper.h>
#include <moveit_task_constructor/subtasks/generate_grasp_pose.h>
#include <moveit_task_constructor/subtasks/cartesian_position_motion.h>

#include <ros/ros.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "plan_pick");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;

	t.addStart( std::make_shared<subtasks::CurrentState>("current state") );

	{
		auto move= std::make_shared<subtasks::Gripper>("open gripper");
		move->setGroup("gripper");
		move->setTo("open");
		t.addAfter(move);
	}

/*
	{
		auto move= std::make_shared<subtask::Move>("move to pre-grasp");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setPlanningTime(5.0);
		t.addAfter(move);
	}
*/

	{
		auto move= std::make_shared<subtasks::CartesianPositionMotion>("proceed to grasp pose");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(.03, 0.1);
		move->setCartesianStepSize(0.02);

		geometry_msgs::PointStamped target;
		target.header.frame_id= "object";
		move->towards(target);
		t.addAfter(move);
	}

	{
		auto gengrasp= std::make_shared<subtasks::GenerateGraspPose>("generate grasp pose");
		gengrasp->setEndEffector("gripper");
		//gengrasp->setGroup("arm");
		gengrasp->setGripperGraspPose("open");
		gengrasp->setObject("object");
		gengrasp->setGraspOffset(.03);
		gengrasp->setAngleDelta(.2);
		t.addAfter(gengrasp);
	}

	{
		auto move= std::make_shared<subtasks::Gripper>("grasp");
		move->setGroup("gripper");
		move->setTo("closed");
		move->graspObject("object");
		t.addAfter(move);
	}

	{
		auto move= std::make_shared<subtasks::CartesianPositionMotion>("lift object");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(0.03, 0.05);
		move->setCartesianStepSize(0.01);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id= "world";
		direction.vector.z= 1.0;
		move->along(direction);
		t.addAfter(move);
	}

	t.plan();

	//t.printState();

	return 0;
}
