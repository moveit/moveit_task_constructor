#include <moveit_task_constructor/introspection.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

Introspection::Introspection()
{
	ros::NodeHandle n;
	task_description_publisher_ = n.advertise<moveit_task_constructor::TaskDescription>(DESCRIPTION_TOPIC, 1);
	task_statistics_publisher_ = n.advertise<moveit_task_constructor::TaskStatistics>(STATISTICS_TOPIC, 1);
	solution_publisher_ = n.advertise<::moveit_task_constructor::Solution>(SOLUTION_TOPIC, 1, true);

	n = ros::NodeHandle("~"); // services are advertised in private namespace
	get_solution_service_ = n.advertiseService("get_solution", &Introspection::getSolution, this);
}

Introspection &Introspection::instance()
{
	static Introspection instance_;
	return instance_;
}

void Introspection::publishTaskDescription(const Task &t)
{
	::moveit_task_constructor::TaskDescription msg;
	task_description_publisher_.publish(t.fillTaskDescription(msg));
}

void Introspection::publishTaskState(const Task &t)
{
	::moveit_task_constructor::TaskStatistics msg;
	task_statistics_publisher_.publish(t.fillTaskStatistics(msg));
}

void Introspection::publishSolution(const SolutionBase &s)
{
	::moveit_task_constructor::Solution msg;
	s.fillMessage(msg);
	publishSolution(msg);
}

bool Introspection::getSolution(moveit_task_constructor::GetSolution::Request  &req,
                                moveit_task_constructor::GetSolution::Response &res)
{
	::moveit_task_constructor::Solution msg;
	const SolutionBase& solution = Repository<SolutionBase>::instance()[req.solution_id];
	solution.fillMessage(msg);
	res.solution = msg;
	return true;
}

void publishAllPlans(const Task &task, bool wait) {
	Task::SolutionProcessor processor
	      = [wait](const ::moveit_task_constructor::Solution& msg, double cost) {
		std::cout << "publishing solution with cost: " << cost << std::endl;
		Introspection::instance().publishSolution(msg);
		if (wait) {
			std::cout << "Press <Enter> to continue ..." << std::endl;
			int ch = getchar();
			if (ch == 'q' || ch == 'Q')
				return false;
		}
		return true;
	};

	task.processSolutions(processor);
}

} }
