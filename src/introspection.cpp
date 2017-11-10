#include "stage_p.h"
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
	stage_to_id_map_[&task_] = 0; // root is task having ID = 0

	task_description_publisher_ = nh_.advertise<moveit_task_constructor::TaskDescription>(DESCRIPTION_TOPIC, 1, true);
	task_statistics_publisher_ = nh_.advertise<moveit_task_constructor::TaskStatistics>(STATISTICS_TOPIC, 1);
	solution_publisher_ = nh_.advertise<::moveit_task_constructor::Solution>(SOLUTION_TOPIC, 1, true);

	get_solution_service_ = nh_.advertiseService("get_solution", &Introspection::getSolution, this);
}

void Introspection::publishTaskDescription()
{
	::moveit_task_constructor::TaskDescription msg;
	task_description_publisher_.publish(fillTaskDescription(msg));
}

void Introspection::publishTaskState()
{
	::moveit_task_constructor::TaskStatistics msg;
	task_statistics_publisher_.publish(fillTaskStatistics(msg));
}

void Introspection::reset()
{
	// send empty task description message to indicate reset
	::moveit_task_constructor::TaskDescription msg;
	msg.id = task_.id();
	task_description_publisher_.publish(msg);

	// reset maps
	stage_to_id_map_.clear();
	stage_to_id_map_[&task_] = 0; // root is task having ID = 0

	id_solution_bimap_.clear();
}

void Introspection::publishSolution(const SolutionBase &s)
{
	::moveit_task_constructor::Solution msg;
	s.fillMessage(msg, this);
	msg.task_id = task_.id();
	s.start()->scene()->getPlanningSceneMsg(msg.start_scene);
	solution_publisher_.publish(msg);
}

void Introspection::publishAllSolutions(bool wait) const
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
	auto it = id_solution_bimap_.left.find(req.solution_id);
	if (it == id_solution_bimap_.left.end())
		return false;

	const SolutionBase& solution = *it->second;
	solution.fillMessage(res.solution, this);
	return true;
}

uint32_t Introspection::stageId(const Stage* const s)
{
	return stage_to_id_map_.insert(std::make_pair(s, stage_to_id_map_.size())).first->second;
}
uint32_t Introspection::stageId(const Stage* const s) const
{
	auto it = stage_to_id_map_.find(s);
	if (it == stage_to_id_map_.end())
		throw std::runtime_error("unknown stage");
	return it->second;
}

uint32_t Introspection::solutionId(const SolutionBase& s)
{
	auto result = id_solution_bimap_.left.insert(std::make_pair(id_solution_bimap_.size(), &s));
	return result.first->first;
}

void Introspection::fillStageStatistics(const Stage& stage, moveit_task_constructor::StageStatistics& s)
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

moveit_task_constructor::TaskDescription& Introspection::fillTaskDescription(moveit_task_constructor::TaskDescription &msg)
{
	ContainerBase::StageCallback stageProcessor =
	      [this, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor::StageDescription desc;
		moveit_task_constructor::StageStatistics stat;
		desc.id = stat.id = stageId(&stage);

		desc.name = stage.name();
		desc.flags = stage.pimpl()->interfaceFlags();
		// TODO fill stage properties

		auto it = stage_to_id_map_.find(stage.pimpl()->parent());
		assert (it != stage_to_id_map_.cend());
		desc.parent_id = stat.parent_id = it->second;

		fillStageStatistics(stage, stat);

		// finally store in msg
		msg.description.push_back(std::move(desc));
		msg.statistics.push_back(std::move(stat));
		return true;
	};

	msg.description.clear();
	msg.statistics.clear();
	task_.stages()->traverseRecursively(stageProcessor);

	msg.id = task_.id();
	return msg;
}

moveit_task_constructor::TaskStatistics& Introspection::fillTaskStatistics(moveit_task_constructor::TaskStatistics &msg)
{
	ContainerBase::StageCallback stageProcessor =
	      [this, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor::StageStatistics stat; // create new Stage msg
		stat.id = stageId(&stage);

		auto it = stage_to_id_map_.find(stage.pimpl()->parent());
		assert (it != stage_to_id_map_.cend());
		stat.parent_id = it->second;

		fillStageStatistics(stage, stat);

		// finally store in msg.stages
		msg.stages.push_back(std::move(stat));
		return true;
	};

	msg.stages.clear();
	task_.stages()->traverseRecursively(stageProcessor);

	msg.id = task_.id();
	return msg;
}

} }
