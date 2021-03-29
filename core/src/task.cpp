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

void swap(StagePrivate*& lhs, StagePrivate*& rhs) {
	// It only makes sense to swap pimpl instances of a Task!
	// However, due to member protection rules, we can only implement it here
	assert(typeid(lhs) == typeid(rhs));

	// swap instances
	::std::swap(lhs, rhs);
	// as well as their me_ pointers
	::std::swap(lhs->me_, rhs->me_);

	// and redirect the parent pointers of children to new parents
	auto& lhs_children = static_cast<ContainerBasePrivate*>(lhs)->children_;
	for (auto it = lhs_children.begin(), end = lhs_children.end(); it != end; ++it) {
		(*it)->pimpl()->unparent();
		(*it)->pimpl()->setParent(static_cast<ContainerBase*>(lhs->me_));
		(*it)->pimpl()->setParentPosition(it);
	}

	auto& rhs_children = static_cast<ContainerBasePrivate*>(rhs)->children_;
	for (auto it = rhs_children.begin(), end = rhs_children.end(); it != end; ++it) {
		(*it)->pimpl()->unparent();
		(*it)->pimpl()->setParent(static_cast<ContainerBase*>(rhs->me_));
		(*it)->pimpl()->setParentPosition(it);
	}
}

const ContainerBase* TaskPrivate::stages() const {
	return children().empty() ? nullptr : static_cast<ContainerBase*>(children().front().get());
}

Task::Task(const std::string& ns, bool introspection, ContainerBase::pointer&& container)
  : WrapperBase(new TaskPrivate(this, ns), std::move(container)) {
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
	swap(this->pimpl_, other.pimpl_);
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

	// provide introspection instance to all stages
	impl->setIntrospection(impl->introspection_.get());
	impl->traverseStages(
	    [impl](Stage& stage, int /*depth*/) {
		    stage.pimpl()->setIntrospection(impl->introspection_.get());
		    return true;
	    },
	    1, UINT_MAX);

	// first time publish task
	if (impl->introspection_)
		impl->introspection_->publishTaskDescription();
}

bool Task::canCompute() const {
	return stages()->canCompute();
}

void Task::compute() {
	stages()->pimpl()->runCompute();
}

bool Task::plan(size_t max_solutions) {
	auto impl = pimpl();
	init();

	impl->preempt_requested_ = false;
	const double available_time = timeout();
	const auto start_time = std::chrono::steady_clock::now();
	while (!impl->preempt_requested_ && canCompute() && (max_solutions == 0 || numSolutions() < max_solutions) &&
	       std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < available_time) {
		compute();
		for (const auto& cb : impl->task_cbs_)
			cb(*this);
		if (impl->introspection_)
			impl->introspection_->publishTaskState();
	}
	printState();
	return numSolutions() > 0;
}

void Task::preempt() {
	pimpl()->preempt_requested_ = true;
}

moveit_msgs::MoveItErrorCodes Task::execute(const SolutionBase& s) {
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution");
	ac.waitForServer();

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	s.fillMessage(goal.solution, pimpl()->introspection_.get());
	s.start()->scene()->getPlanningSceneMsg(goal.solution.start_scene);

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
}  // namespace task_constructor
}  // namespace moveit
