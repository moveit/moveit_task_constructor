#include <moveit_task_constructor/debug.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>

namespace moveit { namespace task_constructor {

bool publishSolution(ros::Publisher& pub,
                     const moveit_task_constructor::Solution &msg,
                     double cost, bool wait){
	std::cout << "publishing solution with cost: " << cost << std::endl;
	pub.publish(msg);
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
	ros::Publisher pub = n.advertise<::moveit_task_constructor::Solution>(topic, 1, true);

	Task::SolutionProcessor processor = std::bind(
		&publishSolution, pub, std::placeholders::_1, std::placeholders::_2, wait);
	task.processSolutions(processor);
}


NewSolutionPublisher::NewSolutionPublisher(const std::string &topic)
{
	ros::NodeHandle n;
	pub_ = n.advertise<::moveit_task_constructor::Solution>(topic, 1, true);
}

void NewSolutionPublisher::operator()(const SolutionBase &s)
{
	// flatten s into vector of SubTrajectories
	::moveit_task_constructor::Solution msg;
	s.fillMessage(msg);

	publishSolution(pub_, msg, s.cost(), false);
}

} }
