#include <moveit_task_constructor/introspection.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

Introspection::Introspection(const std::string &task_topic,
                                               const std::string &solution_topic)
{
	ros::NodeHandle n;
	task_publisher_ = n.advertise<moveit_task_constructor::Task>(task_topic, 1);
	solution_publisher_ = n.advertise<::moveit_task_constructor::Solution>(solution_topic, 1, true);
}

Introspection &Introspection::instance()
{
	static Introspection instance_;
	return instance_;
}

void Introspection::publishTask(const Task &t)
{
	::moveit_task_constructor::Task msg;
	task_publisher_.publish(t.fillMessage(msg));
}

void Introspection::publishSolution(const SolutionBase &s)
{
	::moveit_task_constructor::Solution msg;
	s.fillMessage(msg);
	publishSolution(msg);
}

void Introspection::generateInterfaceService()
{
	Introspection i;
	interfaceState_service_ = n.advertiseService("get_interface_state", &Introspection::getInterfaceState, &i);
}

void Introspection::generateSolutionService()
{
	Introspection i;
	solution_service_ = n.advertiseService("get_solution", &Introspection::getSolution, &i);
}

bool Introspection::getInterfaceState(moveit_task_constructor::GetInterfaceState::Request  &req,
                                      moveit_task_constructor::GetInterfaceState::Response &res)
{
	const InterfaceState& state = Repository<InterfaceState>::instance()[req.state_id];
	moveit_msgs::PlanningScene msg;
	state.scene()->getPlanningSceneDiffMsg(msg);
	res.scene = msg;
	return true;
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
