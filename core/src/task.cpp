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

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/introspection.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <functional>

namespace moveit { namespace task_constructor {

Task::Task(const std::string& id, ContainerBase::pointer &&container)
   : WrapperBase(std::string(), std::move(container)), id_(id)
{
	initModel();

	// monitor state on commandline
	//addTaskCallback(std::bind(&Task::printState, this, std::ref(std::cout)));
	// enable introspection by default
	enableIntrospection(true);
}

void Task::initModel () {
	rml_.reset(new robot_model_loader::RobotModelLoader);
	if( !rml_->getModel() )
		throw Exception("Task failed to construct RobotModel");
}

const moveit::core::RobotModelPtr Task::getRobotModel() const {
	return rml_ ? rml_->getModel() : moveit::core::RobotModelPtr();
}

planning_scene::PlanningScenePtr Task::initScene(ros::Duration timeout) {
	assert(rml_);

	scene_ = std::make_shared<planning_scene::PlanningScene>(rml_->getModel());

	ros::NodeHandle h;
	ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	if (timeout != ros::Duration() && client.waitForExistence(timeout)) {
		moveit_msgs::GetPlanningScene::Request req;
		moveit_msgs::GetPlanningScene::Response res;

		req.components.components =
		      moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
		      | moveit_msgs::PlanningSceneComponents::ROBOT_STATE
		      | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
		      | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
		      | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
		      | moveit_msgs::PlanningSceneComponents::OCTOMAP
		      | moveit_msgs::PlanningSceneComponents::TRANSFORMS
		      | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
		      | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING
		      | moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

		if (client.call(req, res))
			std::const_pointer_cast<planning_scene::PlanningScene>(scene_)->setPlanningSceneMsg(res.scene);
		return scene_;
	}
	ROS_WARN("failed to acquire current PlanningScene, using empty one");
	return scene_;
}

planning_pipeline::PlanningPipelinePtr
Task::createPlanner(const robot_model::RobotModelConstPtr& model, const std::string& ns,
                    const std::string& planning_plugin_param_name,
                    const std::string& adapter_plugins_param_name) {
	typedef std::tuple<std::string, std::string, std::string, std::string> PlannerID;
	static std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline> > planner_cache;

	PlannerID id (model->getName(), ns, planning_plugin_param_name, adapter_plugins_param_name);
	auto it = planner_cache.find(id);

	planning_pipeline::PlanningPipelinePtr planner;
	if (it != planner_cache.cend())
		planner = it->second.lock();
	if (!planner) {
		planner = std::make_shared<planning_pipeline::PlanningPipeline>
		          (model, ros::NodeHandle(ns), planning_plugin_param_name, adapter_plugins_param_name);
		planner_cache[id] = planner;
	}
	return planner;
}

Task::~Task()
{
	reset();
}

void Task::add(pointer &&stage) {
	if (!stage)
		throw std::runtime_error("stage insertion failed: invalid stage pointer");

	if (!stages()->insert(std::move(stage)))
		throw std::runtime_error(std::string("insertion failed for stage: ") + stage->name());
}

void Task::clear()
{
	stages()->clear();
}

void Task::enableIntrospection(bool enable)
{
	if (enable && !introspection_)
		introspection_.reset(new Introspection(*this));
	else if (!enable && introspection_) {
		// reset introspection instance of all stages
		pimpl()->setIntrospection(nullptr);
		pimpl()->traverseStages([this](Stage& stage, int) {
			stage.pimpl()->setIntrospection(nullptr);
			return true;
		}, 1, UINT_MAX);
		introspection_.reset();
	}
}

Introspection &Task::introspection()
{
	enableIntrospection(true);
	return *introspection_;
}

Task::TaskCallbackList::const_iterator Task::addTaskCallback(TaskCallback &&cb)
{
	task_cbs_.emplace_back(std::move(cb));
	return --task_cbs_.cend();
}

void Task::erase(TaskCallbackList::const_iterator which)
{
	task_cbs_.erase(which);
}

void Task::reset()
{
	// signal introspection, that this task was reset
	if (introspection_)
		introspection_->reset();

	WrapperBase::reset();
}

void Task::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	auto impl = pimpl();
	// initialize push connections of wrapped child
	StagePrivate *child = wrapped()->pimpl();
	child->setPrevEnds(impl->pendingBackward());
	child->setNextStarts(impl->pendingForward());

	// explicitly call ContainerBase::init(), not WrapperBase::init()
	// to keep the just set push interface for the wrapped child
	ContainerBase::init(scene);

	// provide introspection instance to all stages
	impl->setIntrospection(introspection_.get());
	impl->traverseStages([this](Stage& stage, int) {
		stage.pimpl()->setIntrospection(introspection_.get());
		return true;
	}, 1, UINT_MAX);

	// first time publish task
	introspection_->publishTaskDescription();
}

bool Task::canCompute() const
{
	return stages()->canCompute();
}

bool Task::compute()
{
	return stages()->compute();
}

bool Task::plan()
{
	reset();
	if (!scene_) initScene();
	init(scene_);

	while(ros::ok() && canCompute()) {
		if (compute()) {
			for (const auto& cb : task_cbs_)
				cb(*this);
			if (introspection_)
				introspection_->publishTaskState();
		} else
			break;
	}
	printState();
	return numSolutions() > 0;
}

size_t Task::numSolutions() const
{
	return stages()->numSolutions();
}

void Task::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	stages()->processSolutions(processor);
}

void Task::publishAllSolutions(bool wait)
{
	enableIntrospection(true);
	introspection_->publishAllSolutions(wait);
}

void Task::onNewSolution(const SolutionBase &s)
{
	// no need to call WrapperBase::onNewSolution!
	if (introspection_)
		introspection_->publishSolution(s);
}

ContainerBase* Task::stages()
{
	return static_cast<ContainerBase*>(WrapperBase::wrapped());
}

const ContainerBase* Task::stages() const
{
	return const_cast<Task*>(this)->stages();
}

PropertyMap &Task::properties()
{
	// forward to wrapped() stage
	return wrapped()->properties();
}

void Task::setProperty(const std::string &name, const boost::any &value)
{
	// forward to wrapped() stage
	wrapped()->setProperty(name, value);
}

std::string Task::id() const
{
	return id_;
}

void Task::printState(std::ostream& os) const
{
	ContainerBase::StageCallback processor = [&os](const Stage& stage, int depth) -> bool {
		os << std::string(2*depth, ' ') << stage << std::endl;
		return true;
	};
	stages()->traverseRecursively(processor);
}

} }
