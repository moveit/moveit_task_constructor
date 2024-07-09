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
#include <moveit_task_constructor_msgs/msg/property.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <moveit/planning_scene/planning_scene.h>

#include <sstream>
#include <boost/bimap.hpp>
#include <rcutils/isalnum_no_locale.h>

static auto LOGGER = rclcpp::get_logger("introspection");

namespace moveit {
namespace task_constructor {

namespace {
std::string getTaskId(const TaskPrivate* task) {
	static const std::string ALLOWED = "_/";
	std::ostringstream oss;
	char our_hostname[256] = { 0 };
	gethostname(our_hostname, sizeof(our_hostname) - 1);
	// Replace all invalid ROS-name chars with an underscore
	std::replace_if(
	    our_hostname, our_hostname + strlen(our_hostname),
	    [](const char ch) { return !rcutils_isalnum_no_locale(ch) && ALLOWED.find(ch) == std::string::npos; }, '_');
	oss << our_hostname << "_" << getpid() << "_" << reinterpret_cast<std::size_t>(task);
	return oss.str();
}
}  // namespace

/**
 * A shared executor between all tasks' introspection, this class will keep track of the number of the added nodes and
 * will only create a spinning thread when we have nodes in the executor, once all the nodes are removed from the
 * executor it will stop the spinning thread, later on if a new node is added a spinning thread will be started again
 */
class IntrospectionExecutor
{
public:
	void add_node(const rclcpp::Node::SharedPtr& node) {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!executor_)
			executor_ = rclcpp::executors::SingleThreadedExecutor::make_unique();
		executor_->add_node(node);
		if (nodes_count_++ == 0)
			executor_thread_ = std::thread([this] { executor_->spin(); });
	}

	void remove_node(const rclcpp::Node::SharedPtr& node) {
		std::lock_guard<std::mutex> lock(mutex_);
		executor_->remove_node(node);
		if (--nodes_count_ == 0) {
			executor_->cancel();
			if (executor_thread_.joinable())
				executor_thread_.join();
			executor_.reset();
		}
	}

private:
	rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
	std::thread executor_thread_;
	size_t nodes_count_ = 0;
	std::mutex mutex_;
};

class IntrospectionPrivate
{
public:
	IntrospectionPrivate(const TaskPrivate* task, Introspection* self) : task_(task), task_id_(getTaskId(task)) {
		node_ = rclcpp::Node::make_shared("introspection_" + task_id_, task_->ns());
		executor_.add_node(node_);
		task_description_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::TaskDescription>(
		    DESCRIPTION_TOPIC, rclcpp::QoS(2).transient_local());
		// send reset message as early as possible to give subscribers time to see it
		indicateReset();

		task_statistics_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::TaskStatistics>(
		    STATISTICS_TOPIC, rclcpp::QoS(1).transient_local());
		solution_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::Solution>(
		    SOLUTION_TOPIC, rclcpp::QoS(1).transient_local());

		get_solution_service_ = node_->create_service<moveit_task_constructor_msgs::srv::GetSolution>(
		    std::string(GET_SOLUTION_SERVICE "_") + task_id_,
		    std::bind(&Introspection::getSolution, self, std::placeholders::_1, std::placeholders::_2));
		resetMaps();
	}
	~IntrospectionPrivate() {
		indicateReset();
		executor_.remove_node(node_);
	}

	void indicateReset() {
		// send empty task description message to indicate reset
		::moveit_task_constructor_msgs::msg::TaskDescription msg;
		msg.task_id = task_id_;
		task_description_publisher_->publish(msg);
	}

	void resetMaps() {
		// reset maps
		stage_to_id_map_.clear();
		stage_to_id_map_[task_] = 0;  // root is task having ID = 0

		id_solution_bimap_.clear();
	}

	/// associated task
	const TaskPrivate* task_;
	const std::string task_id_;

	/// publish task detailed description and current state
	rclcpp::Publisher<moveit_task_constructor_msgs::msg::TaskDescription>::SharedPtr task_description_publisher_;
	rclcpp::Publisher<moveit_task_constructor_msgs::msg::TaskStatistics>::SharedPtr task_statistics_publisher_;
	/// publish new solutions
	rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_publisher_;
	/// services to provide an individual Solution
	rclcpp::Service<moveit_task_constructor_msgs::srv::GetSolution>::SharedPtr get_solution_service_;
	rclcpp::Node::SharedPtr node_;
	IntrospectionExecutor executor_;

	/// mapping from stages to their id
	std::map<const StagePrivate*, moveit_task_constructor_msgs::msg::StageStatistics::_id_type> stage_to_id_map_;
	boost::bimap<uint32_t, const SolutionBase*> id_solution_bimap_;
};

Introspection::Introspection(const TaskPrivate* task) : impl(new IntrospectionPrivate(task, this)) {}

Introspection::~Introspection() {
	delete impl;
}

void Introspection::publishTaskDescription() {
	::moveit_task_constructor_msgs::msg::TaskDescription msg;
	impl->task_description_publisher_->publish(fillTaskDescription(msg));
}

void Introspection::publishTaskState() {
	::moveit_task_constructor_msgs::msg::TaskStatistics msg;
	impl->task_statistics_publisher_->publish(fillTaskStatistics(msg));  // NOLINT(clang-analyzer-cplusplus.Move)
}

void Introspection::reset() {
	impl->indicateReset();
	impl->resetMaps();
}

void Introspection::registerSolution(const SolutionBase& s) {
	solutionId(s);
}

void Introspection::fillSolution(moveit_task_constructor_msgs::msg::Solution& msg, const SolutionBase& s) {
	s.toMsg(msg, this);
	msg.task_id = impl->task_id_;
}

void Introspection::publishSolution(const SolutionBase& s) {
	moveit_task_constructor_msgs::msg::Solution msg;
	fillSolution(msg, s);
	impl->solution_publisher_->publish(msg);
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

bool Introspection::getSolution(const moveit_task_constructor_msgs::srv::GetSolution::Request::SharedPtr& req,
                                const moveit_task_constructor_msgs::srv::GetSolution::Response::SharedPtr& res) {
	const SolutionBase* solution = solutionFromId(req->solution_id);
	if (!solution)
		return false;

	fillSolution(res->solution, *solution);
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
	if (result.second)  // new entry
		RCLCPP_DEBUG_STREAM(LOGGER, "new solution #" << result.first->first << " (" << s.creator()->name()
		                                             << "): " << s.cost() << " " << s.comment());
	return result.first->first;
}

void Introspection::fillStageStatistics(const Stage& stage, moveit_task_constructor_msgs::msg::StageStatistics& s) {
	// successful solutions
	for (const auto& solution : stage.solutions())
		s.solved.push_back(solutionId(*solution));

	// failed solution attempts
	for (const auto& solution : stage.failures())
		s.failed.push_back(solutionId(*solution));

	s.total_compute_time = stage.getTotalComputeTime();
	s.num_failed = stage.numFailures();
}

moveit_task_constructor_msgs::msg::TaskDescription&
Introspection::fillTaskDescription(moveit_task_constructor_msgs::msg::TaskDescription& msg) {
	ContainerBase::StageCallback stage_processor = [this, &msg](const Stage& stage, unsigned int /*depth*/) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::msg::StageDescription desc;
		desc.id = stageId(&stage);
		desc.name = stage.name();
		desc.flags = stage.pimpl()->interfaceFlags();

		// fill stage properties
		for (const auto& pair : stage.properties()) {
			moveit_task_constructor_msgs::msg::Property p;
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

moveit_task_constructor_msgs::msg::TaskStatistics&
Introspection::fillTaskStatistics(moveit_task_constructor_msgs::msg::TaskStatistics& msg) {
	ContainerBase::StageCallback stage_processor = [this, &msg](const Stage& stage, unsigned int /*depth*/) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor_msgs::msg::StageStatistics stat;  // create new Stage msg
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
