#include <ros/ros.h>

#include <moveit_task_constructor/task.h>

#include <moveit_task_constructor/stages/current_state.h>
#include <moveit_task_constructor/stages/gripper.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_plan_gripper");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;

	t.add( std::make_unique<stages::CurrentState>("current state") );
	{
		auto gripper= std::make_unique<stages::Gripper>("close gripper");
		gripper->setEndEffector("gripper");
		gripper->setTo("closed");
		t.add(std::move( gripper ));
	}
	t.plan();

	/*TODO currently not implemented in gripper*/
	/*
	{
		auto gripper= std::make_unique<stages::Gripper>("close gripper");
		gripper->setEndEffector("gripper");
		gripper->setTo("closed");
		t.add(std::move( gripper ));
	}
	t.add( std::make_unique<stages::CurrentState>("current state") );
	t.plan();
	*/
	return 0;
}
