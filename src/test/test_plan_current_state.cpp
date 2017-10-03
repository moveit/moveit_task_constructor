#include <ros/ros.h>

#include <moveit_task_constructor/task.h>

#include <moveit_task_constructor/subtasks/current_state.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_plan_current_state");

	Task t;

	t.add( std::make_unique<subtasks::CurrentState>("current state") );

	t.plan();

	return 0;
}
