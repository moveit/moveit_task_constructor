#include <ros/ros.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "test_plan_current_state");

	Task t;

	t.add( std::make_unique<stages::CurrentState>("current state") );

	t.plan();

	return 0;
}
