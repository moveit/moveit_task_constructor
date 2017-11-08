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

/* Author: Robert Haschke
	Desc:   Monitor manipulation tasks and visualize their solutions
*/

#include "task_display.h"
#include "task_list_model_cache.h"
#include <moveit_task_constructor/introspection.h>
#include <moveit_task_constructor/visualization_tools/task_solution_visualization.h>

#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/status_property.h>

namespace moveit_rviz_plugin
{

TaskDisplay::TaskDisplay() : Display()
{
	robot_description_property_ = new rviz::StringProperty(
	                                 "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
	                                 this, SLOT(changedRobotDescription()), this);

	task_solution_topic_property_ =
	      new rviz::RosTopicProperty("Task Solution Topic", "",
	                                 ros::message_traits::datatype<moveit_task_constructor::Solution>(),
	                                 "The topic on which task solutions (moveit_msgs::Solution messages) are received",
	                                 this, SLOT(changedTaskSolutionTopic()), this);

	trajectory_visual_.reset(new TaskSolutionVisualization(this, this));
}

void TaskDisplay::onInitialize()
{
	Display::onInitialize();
	trajectory_visual_->onInitialize(scene_node_, context_);
}

void TaskDisplay::loadRobotModel()
{
	rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

	if (!rdf_loader_->getURDF())
	{
		this->setStatus(rviz::StatusProperty::Error, "Robot Model",
		                "Failed to load from parameter " + robot_description_property_->getString());
		return;
	}
	this->setStatus(rviz::StatusProperty::Ok, "Robot Model", "Successfully loaded");

	const srdf::ModelSharedPtr& srdf =
	      rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
	robot_model_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));

	// Send to child class
	trajectory_visual_->onRobotModelLoaded(robot_model_);
	trajectory_visual_->onEnable();
}

void TaskDisplay::reset()
{
	Display::reset();
	loadRobotModel();
	trajectory_visual_->reset();
}

void TaskDisplay::onEnable()
{
	Display::onEnable();
	loadRobotModel();

	// (re)initialize task model
	updateTaskListModel();
}

void TaskDisplay::onDisable()
{
	Display::onDisable();
	trajectory_visual_->onDisable();

	// don't monitor topics when disabled
	task_description_sub.shutdown();
	task_statistics_sub.shutdown();
	task_solution_sub.shutdown();
}

void TaskDisplay::update(float wall_dt, float ros_dt)
{
	Display::update(wall_dt, ros_dt);
	mainloop_jobs_.executeJobs();
	trajectory_visual_->update(wall_dt, ros_dt);
}

void TaskDisplay::setName(const QString& name)
{
	BoolProperty::setName(name);
	trajectory_visual_->setName(name);
}

void TaskDisplay::changedRobotDescription()
{
	if (isEnabled())
		reset();
	else
		loadRobotModel();
}

void TaskDisplay::updateTaskListModel()
{
	// generate task monitoring topics from solution topic
	std::string solution_topic = task_solution_topic_property_->getStdString();
	auto lastSep = solution_topic.find_last_of('/');
	std::string base_ns = solution_topic.substr(0, lastSep);

	task_model_ = TaskListModelCache::instance().getModel(base_ns);

	if (task_model_) {
		// listen to task descriptions updates
		boost::function<void(const moveit_task_constructor::TaskDescriptionConstPtr &)> taskDescCB
		      ([this](const moveit_task_constructor::TaskDescriptionConstPtr &msg){
			mainloop_jobs_.addJob([this, msg]() { task_model_->processTaskDescriptionMessage(*msg); });
		});
		task_description_sub = update_nh_.subscribe(base_ns + DESCRIPTION_TOPIC, 2, taskDescCB);

		// listen to task statistics updates
		boost::function<void(const moveit_task_constructor::TaskDescriptionConstPtr &)> taskStatCB
		      ([this](const moveit_task_constructor::TaskDescriptionConstPtr &msg){
			mainloop_jobs_.addJob([this, msg]() { task_model_->processTaskDescriptionMessage(*msg); });
		});
		task_description_sub = update_nh_.subscribe(base_ns + STATISTICS_TOPIC, 2, taskStatCB);
	} else {
		setStatus(rviz::StatusProperty::Error, "Task Monitor", "failed to create TaskListModel");
	}

	// listen to task solutions
	boost::function<void(const moveit_task_constructor::SolutionConstPtr &)> solCB
	      ([this](const moveit_task_constructor::SolutionConstPtr &msg){
		mainloop_jobs_.addJob([this, msg]() {
			if (task_model_) task_model_->processSolutionMessage(*msg);
			// TODO: use already processed trajectory (e.g. by ID)
			trajectory_visual_->showTrajectory(*msg);
		});
	});
	task_solution_sub = update_nh_.subscribe(solution_topic, 2, solCB);

	setStatus(rviz::StatusProperty::Ok, "Task Monitor", "Connected");
}

void TaskDisplay::changedTaskSolutionTopic()
{
	task_description_sub.shutdown();
	task_statistics_sub.shutdown();
	task_solution_sub.shutdown();
	updateTaskListModel();
}

}  // namespace moveit_rviz_plugin
