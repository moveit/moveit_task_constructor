/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner, Robert Haschke */

#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <scope_guard/scope_guard.hpp>

#include <functional>

namespace {
std::string rosNormalizeName(const std::string& name) {
	std::string n;
	n.reserve(name.size());

	// drop invalid initial chars
	auto read = name.begin(), end = name.end();
	while (read != end && !isalpha(*read) && *read != '/' && *read != '~')
		++read;

	// copy (and correct) remaining chars
	while (read != end) {
		char c = *read;
		n.push_back((isalnum(c) || c == '_') ? c : '_');
		++read;
	}
	return n;
}
}  // namespace

namespace moveit {
namespace task_constructor {

TaskPrivate::TaskPrivate(Task* me, const std::string& ns)
  : WrapperBasePrivate(me, std::string()), ns_(rosNormalizeName(ns)), preempt_requested_(false) {}

TaskPrivate& TaskPrivate::operator=(TaskPrivate&& other) {
	this->WrapperBasePrivate::operator=(std::move(other));
	ns_ = std::move(other.ns_);
	robot_model_ = std::move(other.robot_model_);
	robot_model_loader_ = std::move(other.robot_model_loader_);
	task_cbs_ = std::move(other.task_cbs_);
	// Ensure same introspection status, but keep the existing introspection instance,
	// which stores this task pointer and includes it in its task_id_
	static_cast<Task*>(me_)->enableIntrospection(static_cast<bool>(other.introspection_));
	return *this;
}

const ContainerBase* TaskPrivate::stages() const {
	return children().empty() ? nullptr : static_cast<ContainerBase*>(children().front().get());
}

Task::Task(const std::string& ns, bool introspection, ContainerBase::pointer&& container)
  : WrapperBase(new TaskPrivate(this, ns), std::move(container)) {
	setPruning(false);
	setTimeout(std::numeric_limits<double>::max());

	// monitor state on commandline
	// addTaskCallback(std::bind(&Task::printState, this, std::ref(std::cout)));
	// enable introspection by default, but only if ros::init() was called
	if (ros::isInitialized() && introspection)
		enableIntrospection(true);
}

Task::Task(Task&& other)  // NOLINT(performance-noexcept-move-constructor)
  : WrapperBase(new TaskPrivate(this, std::string()), std::make_unique<SerialContainer>()) {
	*this = std::move(other);
}

Task& Task::operator=(Task&& other) {  // NOLINT(performance-noexcept-move-constructor)
	clear();  // remove all stages of current task
	*static_cast<TaskPrivate*>(pimpl_) = std::move(*static_cast<TaskPrivate*>(other.pimpl_));
	return *this;
}

Task::~Task() {
	auto impl = pimpl();
	impl->introspection_.reset();  // stop introspection
	clear();  // remove all stages
	impl->robot_model_.reset();
	// only destroy loader after all references to the model are gone!
	impl->robot_model_loader_.reset();
}

void Task::setRobotModel(const core::RobotModelConstPtr& robot_model) {
	if (!robot_model) {
		ROS_ERROR_STREAM(name() << ": received invalid robot model");
		return;
	}
	auto impl = pimpl();
	if (impl->robot_model_ && impl->robot_model_ != robot_model)
		reset();  // solutions, scenes, etc become invalid
	impl->robot_model_ = robot_model;
}

void Task::loadRobotModel(const std::string& robot_description) {
	auto impl = pimpl();
	impl->robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
	setRobotModel(impl->robot_model_loader_->getModel());
	if (!impl->robot_model_)
		throw Exception("Task failed to construct RobotModel");
}

void Task::add(Stage::pointer&& stage) {
	stages()->add(std::move(stage));
}

void Task::insert(Stage::pointer&& stage, int before) {
	stages()->insert(std::move(stage), before);
}

void Task::clear() {
	reset();
	stages()->clear();
}

void Task::enableIntrospection(bool enable) {
	auto impl = pimpl();
	if (enable && !impl->introspection_)
		impl->introspection_.reset(new Introspection(impl));
	else if (!enable && impl->introspection_) {
		// reset introspection instance of all stages
		impl->setIntrospection(nullptr);
		impl->traverseStages(
		    [](Stage& stage, int /*depth*/) {
			    stage.pimpl()->setIntrospection(nullptr);
			    return true;
		    },
		    1, UINT_MAX);
		impl->introspection_.reset();
	}
}

Introspection& Task::introspection() {
	auto impl = pimpl();
	enableIntrospection(true);
	return *impl->introspection_;
}

Task::TaskCallbackList::const_iterator Task::addTaskCallback(TaskCallback&& cb) {
	auto impl = pimpl();
	impl->task_cbs_.emplace_back(std::move(cb));
	return --(impl->task_cbs_.cend());
}

void Task::eraseTaskCallback(TaskCallbackList::const_iterator which) {
	pimpl()->task_cbs_.erase(which);
}

void Task::reset() {
	auto impl = pimpl();
	// signal introspection, that this task was reset
	if (impl->introspection_)
		impl->introspection_->reset();

	WrapperBase::reset();
}

void Task::init() {
	auto impl = pimpl();
	if (!impl->robot_model_)
		loadRobotModel();

	// initialize push connections of wrapped child
	StagePrivate* child = wrapped()->pimpl();
	child->setPrevEnds(impl->pendingBackward());
	child->setNextStarts(impl->pendingForward());

	// and *afterwards* initialize all children recursively
	stages()->init(impl->robot_model_);
	// task expects its wrapped child to push to both ends, this triggers interface resolution
	stages()->pimpl()->resolveInterface(InterfaceFlags({ GENERATE }));

	// provide introspection instance and preempt_requested to all stages
	auto* introspection = impl->introspection_.get();
	impl->traverseStages(
	    [introspection, impl](Stage& stage, int /*depth*/) {
		    stage.pimpl()->setIntrospection(introspection);
		    stage.pimpl()->setPreemptRequestedMember(&impl->preempt_requested_);
		    return true;
	    },
	    1, UINT_MAX);

	// first time publish task
	if (introspection)
		introspection->publishTaskDescription();
}

bool Task::canCompute() const {
	return stages()->canCompute();
}

void Task::compute() {
	try {
		stages()->pimpl()->runCompute();
	} catch (const PreemptStageException& e) {
		// do nothing, needed for early stop
	}
}

moveit::core::MoveItErrorCode Task::plan(size_t max_solutions) {
	// ensure the preempt request is resetted once this method exits
	auto guard = sg::make_scope_guard([this]() noexcept { this->resetPreemptRequest(); });

	auto impl = pimpl();
	init();

	// Print state and return success if there are solutions otherwise the input error_code
	const auto success_or = [this](const int32_t error_code) -> int32_t {
		if (numSolutions() > 0)
			return moveit::core::MoveItErrorCode::SUCCESS;
		printState();
		explainFailure();
		return error_code;
	};
	const double available_time = timeout();
	const auto start_time = std::chrono::steady_clock::now();
	while (canCompute() && (max_solutions == 0 || numSolutions() < max_solutions)) {
		if (impl->preempt_requested_)
			return success_or(moveit::core::MoveItErrorCode::PREEMPTED);
		if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >= available_time)
			return success_or(moveit::core::MoveItErrorCode::TIMED_OUT);
		compute();
		for (const auto& cb : impl->task_cbs_)
			cb(*this);
		if (impl->introspection_)
			impl->introspection_->publishTaskState();
	};
	return success_or(moveit::core::MoveItErrorCode::PLANNING_FAILED);
}

void Task::preempt() {
	pimpl()->preempt_requested_ = true;
}

void Task::resetPreemptRequest() {
	pimpl()->preempt_requested_ = false;
}

moveit::core::MoveItErrorCode Task::execute(const SolutionBase& s) {
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution");
	if (!ac.waitForServer(ros::Duration(0.5))) {
		ROS_ERROR("Failed to connect to the 'execute_task_solution' action server");
		return moveit::core::MoveItErrorCode::FAILURE;
	}

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	s.toMsg(goal.solution, pimpl()->introspection_.get());

	ac.sendGoal(goal);
	ac.waitForResult();
	return ac.getResult()->error_code;
}

void Task::publishAllSolutions(bool wait) {
	enableIntrospection(true);
	pimpl()->introspection_->publishAllSolutions(wait);
}

void Task::onNewSolution(const SolutionBase& s) {
	// no need to call WrapperBase::onNewSolution!
	auto impl = pimpl();
	for (const auto& cb : impl->solution_cbs_)
		cb(s);
}

ContainerBase* Task::stages() {
	return static_cast<ContainerBase*>(WrapperBase::wrapped());
}

const ContainerBase* Task::stages() const {
	return const_cast<Task*>(this)->stages();
}

PropertyMap& Task::properties() {
	// forward to wrapped() stage
	return wrapped()->properties();
}

void Task::setProperty(const std::string& name, const boost::any& value) {
	// forward to wrapped() stage
	wrapped()->setProperty(name, value);
}

const core::RobotModelConstPtr& Task::getRobotModel() const {
	auto impl = pimpl();
	return impl->robot_model_;
}

void Task::printState(std::ostream& os) const {
	os << *stages();
}

void Task::explainFailure(std::ostream& os) const {
	os << "Failing stage(s):\n";
	stages()->explainFailure(os);
}
}  // namespace task_constructor
}  // namespace moveit
