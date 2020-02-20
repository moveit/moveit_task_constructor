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
}

namespace moveit {
namespace task_constructor {

TaskPrivate::TaskPrivate(Task* me, const std::string& id)
  : WrapperBasePrivate(me, std::string()), id_(rosNormalizeName(id)), preempt_requested_(false) {}

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
	for (auto it = lhs_children.begin(), end = lhs_children.end(); it != end; ++it)
		(*it)->pimpl()->setHierarchy(static_cast<ContainerBase*>(lhs->me_), it);

	auto& rhs_children = static_cast<ContainerBasePrivate*>(rhs)->children_;
	for (auto it = rhs_children.begin(), end = rhs_children.end(); it != end; ++it)
		(*it)->pimpl()->setHierarchy(static_cast<ContainerBase*>(rhs->me_), it);
}

const ContainerBase* TaskPrivate::stages() const {
	return children().empty() ? nullptr : static_cast<ContainerBase*>(children().front().get());
}

Task::Task(const std::string& id, ContainerBase::pointer&& container)
  : WrapperBase(new TaskPrivate(this, id), std::move(container)) {
	if (!id.empty())
		stages()->setName(id);

	// monitor state on commandline
	// addTaskCallback(std::bind(&Task::printState, this, std::ref(std::cout)));
	// enable introspection by default, but only if ros::init() was called
	if (ros::isInitialized())
		enableIntrospection(true);
}

Task::Task(Task&& other) : WrapperBase(new TaskPrivate(this, std::string()), std::make_unique<SerialContainer>()) {
	*this = std::move(other);
}

Task& Task::operator=(Task&& other) {
	clear();  // remove all stages of current task
	swap(this->pimpl_, other.pimpl_);
	return *this;
}

struct PlannerCache
{
	typedef std::tuple<std::string, std::string, std::string> PlannerID;
	typedef std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline>> PlannerMap;
	typedef std::list<std::pair<std::weak_ptr<const robot_model::RobotModel>, PlannerMap>> ModelList;
	ModelList cache_;

	PlannerMap::mapped_type& retrieve(const robot_model::RobotModelConstPtr& model, PlannerID id) {
		// find model in cache_ and remove expired entries while doing so
		ModelList::iterator model_it = cache_.begin();
		while (model_it != cache_.end()) {
			if (model_it->first.expired()) {
				model_it = cache_.erase(model_it);
				continue;
			}
			if (model_it->first.lock() == model)
				break;
			++model_it;
		}
		if (model_it == cache_.end())  // if not found, create a new PlannerMap for this model
			model_it = cache_.insert(cache_.begin(), std::make_pair(model, PlannerMap()));

		return model_it->second.insert(std::make_pair(id, PlannerMap::mapped_type())).first->second;
	}
};

planning_pipeline::PlanningPipelinePtr Task::createPlanner(const robot_model::RobotModelConstPtr& model,
                                                           const std::string& ns,
                                                           const std::string& planning_plugin_param_name,
                                                           const std::string& adapter_plugins_param_name) {
	static PlannerCache cache;
	PlannerCache::PlannerID id(ns, planning_plugin_param_name, adapter_plugins_param_name);

	std::weak_ptr<planning_pipeline::PlanningPipeline>& entry = cache.retrieve(model, id);
	planning_pipeline::PlanningPipelinePtr planner = entry.lock();
	if (!planner) {
		// create new entry
		planner = std::make_shared<planning_pipeline::PlanningPipeline>(
		    model, ros::NodeHandle(ns), planning_plugin_param_name, adapter_plugins_param_name);
		// store in cache
		entry = planner;
	}
	return planner;
}

Task::~Task() {
	auto impl = pimpl();
	clear();  // remove all stages
	impl->robot_model_.reset();
	// only destroy loader after all references to the model are gone!
	impl->robot_model_loader_.reset();
}

void Task::setRobotModel(const core::RobotModelConstPtr& robot_model) {
	auto impl = pimpl();
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
	if (!stage)
		throw std::runtime_error("stage insertion failed: invalid stage pointer");

	if (!stages()->insert(std::move(stage)))
		throw std::runtime_error(std::string("insertion failed for stage: ") + stage->name());
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
		pimpl()->setIntrospection(nullptr);
		pimpl()->traverseStages(
		    [](Stage& stage, int) {
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

void Task::erase(TaskCallbackList::const_iterator which) {
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
	stages()->pimpl()->pruneInterface(InterfaceFlags({ GENERATE }));
	// and *finally* validate connectivity
	stages()->pimpl()->validateConnectivity();

	// provide introspection instance to all stages
	impl->setIntrospection(impl->introspection_.get());
	impl->traverseStages(
	    [impl](Stage& stage, int) {
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
	reset();
	init();

	impl->preempt_requested_ = false;
	while (ros::ok() && !impl->preempt_requested_ && canCompute() &&
	       (max_solutions == 0 || numSolutions() < max_solutions)) {
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

std::string Task::id() const {
	return pimpl()->id();
}

const core::RobotModelConstPtr& Task::getRobotModel() const {
	auto impl = pimpl();
	return impl->robot_model_;
}

void Task::printState(std::ostream& os) const {
	os << *stages();
}
}
}
