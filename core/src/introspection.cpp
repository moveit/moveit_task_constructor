/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke */

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_task_constructor_msgs/Property.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <moveit/planning_scene/planning_scene.h>

#include <sstream>
#include <boost/bimap.hpp>

namespace moveit {
namespace task_constructor {

namespace {
std::string getTaskId(const TaskPrivate* task) {
	std::ostringstream oss;
	char our_hostname[256] = { 0 };
	gethostname(our_hostname, sizeof(our_hostname) - 1);
	// Hostname could have `-` as a character but this is an invalid character in ROS so we replace it with `_`
	std::replace(std::begin(our_hostname), std::end(our_hostname), '-', '_');
	oss << our_hostname << "_" << getpid() << "_" << reinterpret_cast<std::size_t>(task);
	return oss.str();
}
}  // namespace

class IntrospectionPrivate
{
public:
	IntrospectionPrivate(const TaskPrivate* task, Introspection* self)
	  : nh_(std::string("~/") + task->ns())  // topics + services are advertised in private namespace
	  , task_(task)
	  , task_id_(getTaskId(task)) {
		task_description_publisher_ =
		    nh_.advertise<moveit_task_constructor_msgs::TaskDescription>(DESCRIPTION_TOPIC, 2, true);
		// send reset message as early as possible to give subscribers time to see it
		indicateReset();

		task_statistics_publisher_ =
		    nh_.advertise<moveit_task_constructor_msgs::TaskStatistics>(STATISTICS_TOPIC, 1, true);
		solution_publisher_ = nh_.advertise<moveit_task_constructor_msgs::Solution>(SOLUTION_TOPIC, 1, true);

		get_solution_service_ =
		    nh_.advertiseService(std::string(GET_SOLUTION_SERVICE "_") + task_id_, &Introspection::getSolution, self);

		resetMaps();
	}
	~IntrospectionPrivate() { indicateReset(); }

	void indicateReset() {
		// send empty task description message to indicate reset
		::moveit_task_constructor_msgs::TaskDescription msg;
		msg.task_id = task_id_;
		task_description_publisher_.publish(msg);
	}

	void resetMaps() {
		// reset maps
		stage_to_id_map_.clear();
		stage_to_id_map_[task_] = 0;  // root is task having ID = 0

		id_solution_bimap_.clear();
	}

	ros::NodeHandle nh_;
	/// associated task
	const TaskPrivate* task_;
	const std::string task_id_;

	/// publish task detailed description and current state
	ros::Publisher task_description_publisher_;
	ros::Publisher task_statistics_publisher_;
	/// publish new solutions
	ros::Publisher solution_publisher_;
	/// services to provide an individual Solution
	ros::ServiceServer get_solution_service_;

	/// mapping from stages to their id
	std::map<const StagePrivate*, moveit_task_constructor_msgs::StageStatistics::_id_type> stage_to_id_map_;
	boost::bimap<uint32_t, const SolutionBase*> id_solution_bimap_;
};

Introspection::Introspection(const TaskPrivate* task) : impl(new IntrospectionPrivate(task, this)) {}

Introspection::~Introspection() {
	delete impl;
}

void Introspection::publishTaskDescription() {
	::moveit_task_constructor_msgs::TaskDescription msg;
	impl->task_description_publisher_.publish(fillTaskDescription(msg));
}

void Introspection::publishTaskState() {
	::moveit_task_constructor_msgs::TaskStatistics msg;
	impl->task_statistics_publisher_.publish(fillTaskStatistics(msg));
}

void Introspection::reset() {
	impl->indicateReset();
	impl->resetMaps();
}

void Introspection::registerSolution(const SolutionBase& s) {
	solutionId(s);
}

void Introspection::fillSolution(moveit_task_constructor_msgs::Solution& msg, const SolutionBase& s) {
	s.fillMessage(msg, this);
	s.start()->scene()->getPlanningSceneMsg(msg.start_scene);

	msg.task_id = impl->task_id_;
}

void Introspection::publishSolution(const SolutionBase& s) {
	moveit_task_constructor_msgs::Solution msg;
	fillSolution(msg, s);
	impl->solution_publisher_.publish(msg);
}

void Introspection::publishAllSolutions(bool wait) {
	for (const auto& solution : impl->task_->stages()->solutions()) {
		publishSolution(*solution);

		if (wait) {
			std::cout << "Press <Enter> to continue ..." << std::endl;
			int ch = getchar();
			if (ch == 'q' || ch == 'Q')
				break;
		}
	};
}

const SolutionBase* Introspection::solutionFromId(uint id) const {
	auto it = impl->id_solution_bimap_.left.find(id);
	if (it == impl->id_solution_bimap_.left.end())
		return nullptr;
	return it->second;
}

bool Introspection::getSolution(moveit_task_constructor_msgs::GetSolution::Request& req,
                                moveit_task_constructor_msgs::GetSolution::Response& res) {
	const SolutionBase* solution = solutionFromId(req.solution_id);
	if (!solution)
		return false;

	fillSolution(res.solution, *solution);
	return true;
}

uint32_t Introspection::stageId(const Stage* const s) {
	return impl->stage_to_id_map_.insert(std::make_pair(s->pimpl(), impl->stage_to_id_map_.size())).first->second;
}
uint32_t Introspection::stageId(const Stage* const s) const {
	auto it = impl->stage_to_id_map_.find(s->pimpl());
	if (it == impl->stage_to_id_map_.end())
		throw std::runtime_error("unregistered stage: " + s->name());
	return it->second;
}

uint32_t Introspection::solutionId(const SolutionBase& s) {
	auto result = impl->id_solution_bimap_.left.insert(std::make_pair(1 + impl->id_solution_bimap_.size(), &s));
	return result.first->first;
}

void Introspection::fillStageStatistics(const Stage& stage, moveit_task_constructor_msgs::StageStatistics& s) {
	// successful solutions
	for (const auto& solution : stage.solutions())
		s.solved.push_back(solutionId(*solution));

	// failed solution attempts
	for (const auto& solution : stage.failures())
		s.failed.push_back(solutionId(*solution));

	s.total_compute_time = stage.getTotalComputeTime();
	s.num_failed = stage.numFailures();
}

moveit_task_constructor_msgs::TaskDescription&
Introspection::fillTaskDescription(moveit_task_constructor_msgs::TaskDescription& msg) {
	ContainerBase::StageCallback stage_processor = [this, &msg](const Stage& stage, unsigned int /*depth*/) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::StageDescription desc;
		desc.id = stageId(&stage);
		desc.name = stage.name();
		desc.flags = stage.pimpl()->interfaceFlags();

		// fill stage properties
		for (const auto& pair : stage.properties()) {
			moveit_task_constructor_msgs::Property p;
			p.name = pair.first;
			p.description = pair.second.description();
			p.type = pair.second.typeName();
			p.value = pair.second.serialize();
			desc.properties.push_back(p);
		}

		auto it = impl->stage_to_id_map_.find(stage.pimpl()->parent()->pimpl());
		assert(it != impl->stage_to_id_map_.cend());
		desc.parent_id = it->second;

		// finally store in msg
		msg.stages.push_back(std::move(desc));
		return true;
	};

	msg.stages.clear();
	impl->task_->stages()->traverseRecursively(stage_processor);

	msg.task_id = impl->task_id_;
	return msg;
}

moveit_task_constructor_msgs::TaskStatistics&
Introspection::fillTaskStatistics(moveit_task_constructor_msgs::TaskStatistics& msg) {
	ContainerBase::StageCallback stage_processor = [this, &msg](const Stage& stage, unsigned int /*depth*/) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::StageStatistics stat;  // create new Stage msg
		stat.id = stageId(&stage);
		fillStageStatistics(stage, stat);

		// finally store in msg.stages
		msg.stages.push_back(std::move(stat));
		return true;
	};

	msg.stages.clear();
	impl->task_->stages()->traverseRecursively(stage_processor);

	msg.task_id = impl->task_id_;
	return msg;
}
}  // namespace task_constructor
}  // namespace moveit
