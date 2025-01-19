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
#include "task_panel.h"
#include "task_list_model.h"
#include "meta_task_list_model.h"
#include <moveit/task_constructor/introspection.h>
#include <moveit/visualization_tools/task_solution_visualization.h>
#include <moveit/visualization_tools/marker_visualization.h>
#include <moveit/visualization_tools/display_solution.h>
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>

#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <OgreSceneNode.h>
#include <QTimer>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.task_display");

namespace moveit_rviz_plugin {

TaskDisplay::TaskDisplay() : Display(), panel_requested_(false), received_task_description_(false) {
	task_list_model_.reset(new TaskListModel);

	MetaTaskListModel::instance().insertModel(task_list_model_.get(), this);

	connect(task_list_model_.get(), SIGNAL(rowsInserted(QModelIndex, int, int)), this,
	        SLOT(onTasksInserted(QModelIndex, int, int)));
	connect(task_list_model_.get(), SIGNAL(rowsAboutToBeRemoved(QModelIndex, int, int)), this,
	        SLOT(onTasksRemoved(QModelIndex, int, int)));
	connect(task_list_model_.get(), SIGNAL(dataChanged(QModelIndex, QModelIndex)), this,
	        SLOT(onTaskDataChanged(QModelIndex, QModelIndex)));

	robot_description_property_ = new rviz_common::properties::StringProperty(
	    "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
	    this, SLOT(changedRobotDescription()), this);

	task_solution_topic_property_ = new rviz_common::properties::RosTopicProperty(
	    "Task Solution Topic", "", rosidl_generator_traits::data_type<moveit_task_constructor_msgs::msg::Solution>(),
	    "The topic on which task solutions (moveit_msgs::msg::Solution messages) are received", this,
	    SLOT(changedTaskSolutionTopic()), this);

	trajectory_visual_.reset(new TaskSolutionVisualization(this, this));
	connect(trajectory_visual_.get(), SIGNAL(activeStageChanged(size_t)), task_list_model_.get(),
	        SLOT(highlightStage(size_t)));

	tasks_property_ =
	    new rviz_common::properties::Property("Tasks", QVariant(), "Tasks received on monitored topic", this);
}

TaskDisplay::~TaskDisplay() {
	if (panel_requested_)
		TaskPanel::release();  // Indicate that we don't need a TaskPanel anymore
}

void TaskDisplay::onInitialize() {
	Display::onInitialize();
	rviz_ros_node_ = context_->getRosNodeAbstraction();
	task_solution_topic_property_->initialize(rviz_ros_node_);
	trajectory_visual_->onInitialize(scene_node_, context_);
	task_list_model_->setDisplayContext(context_);
}

inline void TaskDisplay::requestPanel() {
	if (panel_requested_)
		return;  // already done

	// Create a new TaskPanel if not yet done.
	// This cannot be done in initialize(), because Panel loading follows Display loading in rviz.
	panel_requested_ = true;
	TaskPanel::request(context_->getWindowManager());
}

void TaskDisplay::loadRobotModel() {
	rdf_loader_.reset(
	    new rdf_loader::RDFLoader(rviz_ros_node_.lock()->get_raw_node(), robot_description_property_->getStdString()));

	if (!rdf_loader_->getURDF()) {
		this->setStatus(rviz_common::properties::StatusProperty::Error, "Robot Model",
		                "Failed to load from parameter " + robot_description_property_->getString());
		return;
	}
	this->setStatus(rviz_common::properties::StatusProperty::Ok, "Robot Model", "Successfully loaded");

	const srdf::ModelSharedPtr& srdf =
	    rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
	robot_model_.reset(new moveit::core::RobotModel(rdf_loader_->getURDF(), srdf));

	// Send to child class
	trajectory_visual_->onRobotModelLoaded(robot_model_);
	trajectory_visual_->onEnable();

	// share the planning scene with task models
	task_list_model_->setScene(trajectory_visual_->getScene());

	// perform any postponed subscription to topics (after scene is well-defined)
	changedTaskSolutionTopic();
}

void TaskDisplay::reset() {
	Display::reset();
	loadRobotModel();
	trajectory_visual_->reset();
}

void TaskDisplay::save(rviz_common::Config config) const {
	Display::save(config);
}

void TaskDisplay::load(const rviz_common::Config& config) {
	Display::load(config);
}

void TaskDisplay::onEnable() {
	Display::onEnable();
	loadRobotModel();
	calculateOffsetPosition();
}

void TaskDisplay::onDisable() {
	Display::onDisable();
	trajectory_visual_->onDisable();
}

void TaskDisplay::fixedFrameChanged() {
	Display::fixedFrameChanged();
	calculateOffsetPosition();
}

void TaskDisplay::calculateOffsetPosition() {
	if (!robot_model_)
		return;

	Ogre::Vector3 position;
	Ogre::Quaternion orientation;

	context_->getFrameManager()->getTransform(robot_model_->getModelFrame(), rclcpp::Time(0, 0, RCL_ROS_TIME), position,
	                                          orientation);

	scene_node_->setPosition(position);
	scene_node_->setOrientation(orientation);
}

void TaskDisplay::update(float wall_dt, float ros_dt) {
	requestPanel();
	Display::update(wall_dt, ros_dt);
	calculateOffsetPosition();
	trajectory_visual_->update(wall_dt, ros_dt);
}

void TaskDisplay::changedRobotDescription() {
	if (isEnabled())
		reset();
	else
		loadRobotModel();
}

void TaskDisplay::taskDescriptionCB(const moveit_task_constructor_msgs::msg::TaskDescription::ConstSharedPtr& msg) {
	setStatus(rviz_common::properties::StatusProperty::Ok, "Task Monitor", "OK");
	requestPanel();
	task_list_model_->processTaskDescriptionMessage(*msg, base_ns_ + GET_SOLUTION_SERVICE "_" + msg->task_id);

	// Start listening to other topics if this is the first description
	// Waiting for the description ensures we do not receive data that cannot be interpreted yet
	if (!received_task_description_ && !msg->stages.empty()) {
		auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
		if (!ros_node_abstraction) {
			RCLCPP_INFO(LOGGER, "Unable to lock weak_ptr from DisplayContext in taskDescriptionCB");
			return;
		}
		auto node = ros_node_abstraction->get_raw_node();
		received_task_description_ = true;
		task_statistics_sub = node->create_subscription<moveit_task_constructor_msgs::msg::TaskStatistics>(
		    base_ns_ + STATISTICS_TOPIC, rclcpp::QoS(2).transient_local(),
		    std::bind(&TaskDisplay::taskStatisticsCB, this, std::placeholders::_1));
		task_solution_sub = node->create_subscription<moveit_task_constructor_msgs::msg::Solution>(
		    base_ns_ + SOLUTION_TOPIC, rclcpp::QoS(2).transient_local(),
		    std::bind(&TaskDisplay::taskSolutionCB, this, std::placeholders::_1));
	}
}

void TaskDisplay::taskStatisticsCB(const moveit_task_constructor_msgs::msg::TaskStatistics::ConstSharedPtr& msg) {
	setStatus(rviz_common::properties::StatusProperty::Ok, "Task Monitor", "OK");
	task_list_model_->processTaskStatisticsMessage(*msg);
}

void TaskDisplay::taskSolutionCB(const moveit_task_constructor_msgs::msg::Solution::ConstSharedPtr& msg) {
	setStatus(rviz_common::properties::StatusProperty::Ok, "Task Monitor", "OK");
	try {
		const DisplaySolutionPtr& s = task_list_model_->processSolutionMessage(*msg);
		if (s)
			trajectory_visual_->showTrajectory(s, false);
		else
			setSolutionStatus(false);
	} catch (const std::invalid_argument& e) {
		RCLCPP_ERROR_STREAM(LOGGER, e.what());
		setSolutionStatus(false, e.what());
	}
}

void TaskDisplay::changedTaskSolutionTopic() {
	// postpone setup until scene is well-defined
	if (!trajectory_visual_->getScene())
		return;

	task_description_sub.reset();
	task_statistics_sub.reset();
	task_solution_sub.reset();

	received_task_description_ = false;

	// generate task monitoring topics from solution topic
	const QString& solution_topic = task_solution_topic_property_->getString();
	if (!solution_topic.endsWith(QString("/").append(SOLUTION_TOPIC))) {
		setStatus(rviz_common::properties::StatusProperty::Error, "Task Monitor",
		          QString("Invalid topic. Expecting a name ending on \"/%1\"").arg(SOLUTION_TOPIC));
		return;
	}

	base_ns_ = solution_topic.toStdString().substr(0, solution_topic.length() - strlen(SOLUTION_TOPIC));

	auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
	if (!ros_node_abstraction) {
		RCLCPP_INFO(LOGGER, "Unable to lock weak_ptr from DisplayContext in changedTaskSolutionTopic");
		return;
	}
	// listen to task descriptions updates
	task_description_sub =
	    ros_node_abstraction->get_raw_node()->create_subscription<moveit_task_constructor_msgs::msg::TaskDescription>(
	        base_ns_ + DESCRIPTION_TOPIC, rclcpp::QoS(10).transient_local(),
	        std::bind(&TaskDisplay::taskDescriptionCB, this, std::placeholders::_1));

	setStatus(rviz_common::properties::StatusProperty::Warn, "Task Monitor", "No messages received");
}

void TaskDisplay::setSolutionStatus(bool ok, const char* msg) {
	if (ok)
		setStatus(rviz_common::properties::StatusProperty::Ok, "Solution", "Ok");
	else
		setStatus(rviz_common::properties::StatusProperty::Warn, "Solution", msg ? msg : "Retrieval failed");
}

void TaskDisplay::onTasksInserted(const QModelIndex& parent, int first, int last) {
	if (parent.isValid())
		return;  // only handle top-level items

	TaskListModel* m = static_cast<TaskListModel*>(sender());
	for (; first <= last; ++first) {
		QModelIndex idx = m->index(first, 0, parent);
		tasks_property_->addChild(
		    new rviz_common::properties::Property(idx.data().toString(), idx.sibling(idx.row(), 1).data()), first);
	}
}

void TaskDisplay::onTasksRemoved(const QModelIndex& parent, int first, int last) {
	if (parent.isValid())
		return;  // only handle top-level items

	for (; first <= last; ++first)
		delete tasks_property_->takeChildAt(first);

	trajectory_visual_->reset();
}

void TaskDisplay::onTaskDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight) {
	if (topLeft.parent().isValid())
		return;  // only handle top-level items

	for (int row = topLeft.row(); row <= bottomRight.row(); ++row) {
		rviz_common::properties::Property* child = tasks_property_->childAt(row);
		assert(child);

		if (topLeft.column() <= 0 && 0 <= bottomRight.column())  // name changed
			child->setName(topLeft.sibling(row, 0).data().toString());
		if (topLeft.column() <= 1 && 1 <= bottomRight.column())  // #solutions changed
			child->setValue(topLeft.sibling(row, 1).data());
	}
}

}  // namespace moveit_rviz_plugin
