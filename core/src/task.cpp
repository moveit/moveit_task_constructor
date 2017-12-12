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
   : WrapperBase(std::string()), id_(id)
{
	task_starts_.reset(new Interface(Interface::NotifyFunction()));
	task_ends_.reset(new Interface(Interface::NotifyFunction()));

	insert(std::move(container));
	initModel();

	// monitor state on commandline
	addTaskCallback(&printState);
	// enable introspection by default
	enableIntrospection(true);
}

void Task::initModel () {
	rml_.reset(new robot_model_loader::RobotModelLoader);
	if( !rml_->getModel() )
		throw Exception("Task failed to construct RobotModel");
}
void Task::initScene() {
	assert(rml_);

	ros::NodeHandle h;
	ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	client.waitForExistence();

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

	if(!client.call(req, res)){
		throw Exception("Task failed to acquire current PlanningScene");
	}

	scene_ = std::make_shared<planning_scene::PlanningScene>(rml_->getModel());
	std::const_pointer_cast<planning_scene::PlanningScene>(scene_)->setPlanningSceneMsg(res.scene);
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
	else if (!enable && introspection_)
		introspection_.reset();
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

	task_starts_->clear();
	task_ends_->clear();
	WrapperBase::reset();

	// connect my prevEnds() / nextStarts as WrapperBase will refer to them
	auto impl = pimpl();
	impl->setPrevEnds(task_ends_);
	impl->setNextStarts(task_starts_);
}

void Task::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	WrapperBase::init(scene);
	if (introspection_)
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
	initScene();
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
	return numSolutions() > 0;
}

size_t Task::numSolutions() const
{
	auto w = stages();
	// during initial insert() we call numSolutions(), but wrapped() is not yet defined
	return w ? w->numSolutions() : 0;
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

void Task::onNewSolution(SolutionBase &s)
{
	pimpl()->newSolution(s);
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

std::string Task::id() const
{
	return id_;
}

void Task::printState(const Task &t){
	ContainerBase::StageCallback processor = [](const Stage& stage, int depth) -> bool {
		std::cout << std::string(2*depth, ' ') << stage << std::endl;
		return true;
	};
	t.stages()->traverseRecursively(processor);
}

} }
