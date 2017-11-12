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

	task_monitor_topic_property_ =
	      new rviz::RosTopicProperty("Task Monitor Topic", DEFAULT_TASK_MONITOR_TOPIC,
	                                 ros::message_traits::datatype<moveit_task_constructor::Task>(),
	                                 "The topic on which task updates (moveit_msgs::Task messages) are received",
	                                 this, SLOT(changedTaskMonitorTopic()), this);

	task_solution_topic_property_ =
	      new rviz::RosTopicProperty("Task Solution Topic", DEFAULT_TASK_SOLUTION_TOPIC,
	                                 ros::message_traits::datatype<moveit_task_constructor::Solution>(),
	                                 "The topic on which task solutions (moveit_msgs::Solution messages) are received",
	                                 this, SLOT(changedTaskSolutionTopic()), this);


	trajectory_visual_.reset(new TaskSolutionVisualization(this, this));
	tasks_property_ =
	      new rviz::Property("Tasks", QVariant(), "Tasks received on monitored topic", this);
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
	task_monitor_sub.shutdown();
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
	if (task_list_model_) {
		disconnect(task_list_model_.get(), &TaskListModel::rowsInserted, this, &TaskDisplay::onTasksInserted);
		disconnect(task_list_model_.get(), &TaskListModel::rowsAboutToBeRemoved, this, &TaskDisplay::onTasksRemoved);
	}
	tasks_property_->removeChildren();

	task_list_model_ = TaskListModelCache::instance().getModel(
	                 task_monitor_topic_property_->getString(),
	                 task_solution_topic_property_->getString());

	if (!task_list_model_) {
		if (task_monitor_topic_property_->getString().isEmpty())
			setStatus(rviz::StatusProperty::Warn, "Task Monitor", "invalid task monitor topic");
		else if (task_solution_topic_property_->getString().isEmpty())
			setStatus(rviz::StatusProperty::Warn, "Task Monitor", "invalid task solution topic");
		else
			setStatus(rviz::StatusProperty::Error, "Task Monitor", "failed to create TaskListModel");
	} else {
		boost::function<void(const moveit_task_constructor::TaskConstPtr &)> taskCB
		      ([this](const moveit_task_constructor::TaskConstPtr &msg){
			mainloop_jobs_.addJob([this, msg]() { task_list_model_->processTaskMessage(*msg); });
		});
		task_monitor_sub = update_nh_.subscribe(task_monitor_topic_property_->getStdString(), 2, taskCB);

		boost::function<void(const moveit_task_constructor::SolutionConstPtr &)> solCB
		      ([this](const moveit_task_constructor::SolutionConstPtr &msg){
			mainloop_jobs_.addJob([this, msg]() { task_list_model_->processSolutionMessage(*msg); });
			// TODO: use already processed trajectory (e.g. by ID)
			mainloop_jobs_.addJob([this, msg]() {
				trajectory_visual_->showTrajectory(*msg);
			});
		});
		task_solution_sub = update_nh_.subscribe(task_solution_topic_property_->getStdString(), 2, solCB);

		setStatus(rviz::StatusProperty::Ok, "Task Monitor", "Connected");

		onTasksInserted(QModelIndex(), 0, task_list_model_->rowCount()-1);
		connect(task_list_model_.get(), &TaskListModel::rowsInserted, this, &TaskDisplay::onTasksInserted);
		connect(task_list_model_.get(), &TaskListModel::rowsAboutToBeRemoved, this, &TaskDisplay::onTasksRemoved);
	}
}

void TaskDisplay::changedTaskMonitorTopic()
{
	task_monitor_sub.shutdown();
	updateTaskListModel();
}

void TaskDisplay::changedTaskSolutionTopic()
{
	task_solution_sub.shutdown();
	updateTaskListModel();
}

void TaskDisplay::onTasksInserted(const QModelIndex &parent, int first, int last)
{
	if (parent.isValid()) return;

	TaskListModel* m = static_cast<TaskListModel*>(sender());
	for (; first <= last; ++first) {
		QModelIndex idx = m->index(first, 0, parent);
		tasks_property_->addChild(new rviz::Property(idx.data().toString(), idx.sibling(idx.row(), 1).data()));
	}
}

void TaskDisplay::onTasksRemoved(const QModelIndex &parent, int first, int last)
{
	if (parent.isValid()) return;

	for (; first <= last; ++first) {
		rviz::Property *child = tasks_property_->takeChildAt(first);
		delete child;
	}
}

}  // namespace moveit_rviz_plugin
