#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/storage.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <moveit/planning_scene/planning_scene.h>

#include <boost/bimap.hpp>

namespace moveit { namespace task_constructor {

class IntrospectionPrivate {
public:
	IntrospectionPrivate(const Task &task)
	   : nh_(std::string("~/") + task.id()) // topics + services are advertised in private namespace
	   , task_(task)
	{
		resetMaps();
		task_description_publisher_ = nh_.advertise<moveit_task_constructor_msgs::TaskDescription>(DESCRIPTION_TOPIC, 1, true);
		task_statistics_publisher_ = nh_.advertise<moveit_task_constructor_msgs::TaskStatistics>(STATISTICS_TOPIC, 1, true);
		solution_publisher_ = nh_.advertise<moveit_task_constructor_msgs::Solution>(SOLUTION_TOPIC, 1, true);
	}
	void resetMaps () {
		// reset maps
		stage_to_id_map_.clear();
		stage_to_id_map_[&task_] = 0; // root is task having ID = 0

		id_solution_bimap_.clear();
	}

	ros::NodeHandle nh_;
	/// associated task
	const Task &task_;

	/// publish task detailed description and current state
	ros::Publisher task_description_publisher_;
	ros::Publisher task_statistics_publisher_;
	/// publish new solutions
	ros::Publisher solution_publisher_;
	/// services to provide an individual Solution
	ros::ServiceServer get_solution_service_;

	/// mapping from stages to their id
	std::map<const void*, moveit_task_constructor_msgs::StageStatistics::_id_type> stage_to_id_map_;
	boost::bimap<uint32_t, const SolutionBase*> id_solution_bimap_;
};

Introspection::Introspection(const Task &task)
   : impl(new IntrospectionPrivate(task))
{
	impl->get_solution_service_ = impl->nh_.advertiseService("get_solution", &Introspection::getSolution, this);
}

Introspection::~Introspection()
{
	delete impl;
}

void Introspection::publishTaskDescription()
{
	::moveit_task_constructor_msgs::TaskDescription msg;
	impl->task_description_publisher_.publish(fillTaskDescription(msg));
}

void Introspection::publishTaskState()
{
	::moveit_task_constructor_msgs::TaskStatistics msg;
	impl->task_statistics_publisher_.publish(fillTaskStatistics(msg));
}

void Introspection::reset()
{
	// send empty task description message to indicate reset
	::moveit_task_constructor_msgs::TaskDescription msg;
	msg.id = impl->task_.id();
	impl->task_description_publisher_.publish(msg);

	impl->resetMaps();
}

void Introspection::fillSolution(moveit_task_constructor_msgs::Solution &msg,
                                 const SolutionBase &s)
{
	s.fillMessage(msg, this);
	msg.task_id = impl->task_.id();
	s.start()->scene()->getPlanningSceneMsg(msg.start_scene);
}

void Introspection::publishSolution(const SolutionBase &s)
{
	moveit_task_constructor_msgs::Solution msg;
	fillSolution(msg, s);
	impl->solution_publisher_.publish(msg);
}

void Introspection::publishAllSolutions(bool wait)
{
	moveit_task_constructor_msgs::Solution msg;

	Stage::SolutionProcessor processor = [this, &msg, wait](const SolutionBase& s) {
		std::cout << "publishing solution with cost: " << s.cost() << std::endl;
		msg.sub_solution.clear();
		msg.sub_trajectory.clear();
		s.fillMessage(msg, this);
		msg.task_id = impl->task_.id();
		s.start()->scene()->getPlanningSceneMsg(msg.start_scene);
		impl->solution_publisher_.publish(msg);

		if (wait) {
			std::cout << "Press <Enter> to continue ..." << std::endl;
			int ch = getchar();
			if (ch == 'q' || ch == 'Q')
				return false;
		}
		return true;
	};

	impl->task_.processSolutions(processor);
}

bool Introspection::getSolution(moveit_task_constructor_msgs::GetSolution::Request  &req,
                                moveit_task_constructor_msgs::GetSolution::Response &res)
{
	auto it = impl->id_solution_bimap_.left.find(req.solution_id);
	if (it == impl->id_solution_bimap_.left.end())
		return false;

	fillSolution(res.solution, *it->second);
	return true;
}

uint32_t Introspection::stageId(const Stage* const s)
{
	return impl->stage_to_id_map_.insert(std::make_pair(s, impl->stage_to_id_map_.size())).first->second;
}
uint32_t Introspection::stageId(const Stage* const s) const
{
	auto it = impl->stage_to_id_map_.find(s);
	if (it == impl->stage_to_id_map_.end())
		throw std::runtime_error("unknown stage");
	return it->second;
}

uint32_t Introspection::solutionId(const SolutionBase& s)
{
	auto result = impl->id_solution_bimap_.left.insert(std::make_pair(1 + impl->id_solution_bimap_.size(), &s));
	return result.first->first;
}

void Introspection::fillStageStatistics(const Stage& stage, moveit_task_constructor_msgs::StageStatistics& s)
{
	// successfull solutions
	Stage::SolutionProcessor solutionProcessor = [this, &s](const SolutionBase& solution) {
		s.solved.push_back(solutionId(solution));
		return true;
	};
	stage.processSolutions(solutionProcessor);

	// failed solution attempts
	solutionProcessor = [this, &s](const SolutionBase& solution) {
		s.failed.push_back(solutionId(solution));
		return true;
	};
	stage.processFailures(solutionProcessor);
}

moveit_task_constructor_msgs::TaskDescription& Introspection::fillTaskDescription(moveit_task_constructor_msgs::TaskDescription &msg)
{
	ContainerBase::StageCallback stageProcessor =
	      [this, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::StageDescription desc;
		desc.id = stageId(&stage);
		desc.name = stage.name();
		desc.flags = stage.pimpl()->interfaceFlags();
		// TODO fill stage properties

		auto it = impl->stage_to_id_map_.find(stage.pimpl()->parent());
		assert (it != impl->stage_to_id_map_.cend());
		desc.parent_id = it->second;

		// finally store in msg
		msg.stages.push_back(std::move(desc));
		return true;
	};

	msg.stages.clear();
	impl->task_.stages()->traverseRecursively(stageProcessor);

	msg.id = impl->task_.id();
	return msg;
}

moveit_task_constructor_msgs::TaskStatistics& Introspection::fillTaskStatistics(moveit_task_constructor_msgs::TaskStatistics &msg)
{
	ContainerBase::StageCallback stageProcessor =
	      [this, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::StageStatistics stat; // create new Stage msg
		stat.id = stageId(&stage);
		fillStageStatistics(stage, stat);

		// finally store in msg.stages
		msg.stages.push_back(std::move(stat));
		return true;
	};

	msg.stages.clear();
	impl->task_.stages()->traverseRecursively(stageProcessor);

	msg.id = impl->task_.id();
	return msg;
}

} }
