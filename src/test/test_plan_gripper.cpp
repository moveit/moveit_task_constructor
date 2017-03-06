#include <ros/ros.h>

#include <moveit_task_constructor/task.h>

#include <moveit_task_constructor/subtasks/current_state.h>
#include <moveit_task_constructor/subtasks/gripper.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_plan_gripper");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;

	t.addStart( std::make_shared<subtasks::CurrentState>("current state") );
	{
		auto gripper= std::make_shared<subtasks::Gripper>("close gripper");
		gripper->setEndEffector("gripper");
		gripper->setTo("closed");
		t.addAfter( gripper );
	}
	t.plan();

	/*TODO currently not implemented in gripper*/
	/*
	{
		auto gripper= std::make_shared<subtasks::Gripper>("close gripper");
		gripper->setEndEffector("gripper");
		gripper->setTo("closed");
		t.addStart( gripper );
	}
	t.addAfter( std::make_shared<subtasks::CurrentState>("current state") );
	t.plan();
	*/
	return 0;
}
