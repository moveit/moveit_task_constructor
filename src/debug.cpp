#include <moveit_task_constructor/debug.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

bool publishSolution(ros::Publisher& pub, const SolutionTrajectory& solution, double cost, bool wait){
	if (solution.empty())
		return true;

	moveit_msgs::DisplayTrajectory dt;

	const robot_state::RobotState& state = solution.front()->start()->scene()->getCurrentState();
	robot_state::robotStateToRobotStateMsg(state, dt.trajectory_start);
	dt.model_id = state.getRobotModel()->getName();

	for(const SubTrajectory* t : solution){
		if(t->trajectory()){
			dt.trajectory.emplace_back();
			t->trajectory()->getRobotTrajectoryMsg(dt.trajectory.back());
		}
	}

	std::cout << "publishing solution with cost: " << cost << std::endl;
	pub.publish(dt);
	if (wait) {
		std::cout << "Press <Enter> to continue ..." << std::endl;
		int ch = getchar();
		if (ch == 'q' || ch == 'Q')
			return false;
	}
	return true;
}

void publishAllPlans(const Task &task, const std::string &topic, bool wait) {
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<moveit_msgs::DisplayTrajectory>(topic, 1, true);

	Task::SolutionProcessor processor = std::bind(
		&publishSolution, pub, std::placeholders::_1, std::placeholders::_2, wait);
	task.processSolutions(processor);
}


NewSolutionPublisher::NewSolutionPublisher(const std::string &topic)
{
	ros::NodeHandle n;
	pub_ = n.advertise<moveit_msgs::DisplayTrajectory>(topic, 1, true);
}

void NewSolutionPublisher::operator()(const SolutionBase &s)
{
	// flatten s into vector of SubTrajectories
	SolutionTrajectory solution;
	s.flattenTo(solution);

	publishSolution(pub_, solution, s.cost(), false);
}

} }
