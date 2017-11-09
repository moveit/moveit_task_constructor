#include <moveit_task_constructor/introspection.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

Introspection::Introspection(const Task &task)
   : nh_(std::string("~/") + task.id()) // topics + services are advertised in private namespace
   , task_(task)
{
	task_description_publisher_ = nh_.advertise<moveit_task_constructor::TaskDescription>(DESCRIPTION_TOPIC, 1, true);
	task_statistics_publisher_ = nh_.advertise<moveit_task_constructor::TaskStatistics>(STATISTICS_TOPIC, 1);
	solution_publisher_ = nh_.advertise<::moveit_task_constructor::Solution>(SOLUTION_TOPIC, 1, true);

	get_solution_service_ = nh_.advertiseService("get_solution", &Introspection::getSolution, this);
}

void Introspection::publishTaskDescription()
{
	::moveit_task_constructor::TaskDescription msg;
	task_description_publisher_.publish(task_.fillTaskDescription(msg));
}

void Introspection::publishTaskState()
{
	::moveit_task_constructor::TaskStatistics msg;
	task_statistics_publisher_.publish(task_.fillTaskStatistics(msg));
}

void Introspection::reset()
{
	::moveit_task_constructor::TaskDescription msg;
	task_description_publisher_.publish(msg);
}

void Introspection::publishSolution(const SolutionBase &s)
{
	::moveit_task_constructor::Solution msg;
	s.fillMessage(msg);
	solution_publisher_.publish(msg);
}

void Introspection::publishAllSolutions(bool wait)
{
	Task::SolutionProcessor processor
	      = [this, wait](const ::moveit_task_constructor::Solution& msg, double cost) {
		std::cout << "publishing solution with cost: " << cost << std::endl;
		solution_publisher_.publish(msg);
		if (wait) {
			std::cout << "Press <Enter> to continue ..." << std::endl;
			int ch = getchar();
			if (ch == 'q' || ch == 'Q')
				return false;
		}
		return true;
	};

	task_.processSolutions(processor);
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

} }
