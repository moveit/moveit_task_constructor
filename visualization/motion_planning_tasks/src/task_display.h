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

#pragma once

#include <rviz/display.h>
#include <moveit/visualization_tools/task_solution_visualization.h>

#ifndef Q_MOC_RUN
#include "job_queue.h"
#include <moveit/macros/class_forward.h>
#include <ros/subscriber.h>
#include <moveit_task_constructor_msgs/TaskDescription.h>
#include <moveit_task_constructor_msgs/TaskStatistics.h>
#include <moveit_task_constructor_msgs/Solution.h>
#endif

namespace rviz {
class StringProperty;
class RosTopicProperty;
}  // namespace rviz

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}
}  // namespace moveit
namespace rdf_loader {
MOVEIT_CLASS_FORWARD(RDFLoader);
}

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(DisplaySolution);
class TaskListModel;

class TaskDisplay : public rviz::Display
{
	Q_OBJECT

public:
	TaskDisplay();
	~TaskDisplay() override;

	void loadRobotModel();

	void update(float wall_dt, float ros_dt) override;
	void reset() override;
	void save(rviz::Config config) const override;
	void load(const rviz::Config& config) override;

	void setName(const QString& name) override;
	void setSolutionStatus(bool ok, const char* msg = "");

	TaskListModel& getTaskListModel() { return *task_list_model_; }
	TaskSolutionVisualization* visualization() const { return trajectory_visual_.get(); }

	inline void clearMarkers() { trajectory_visual_->clearMarkers(); }
	inline void addMarkers(const DisplaySolutionPtr& s) { trajectory_visual_->addMarkers(s); }

protected:
	void onInitialize() override;
	void onEnable() override;
	void onDisable() override;
	void fixedFrameChanged() override;
	void calculateOffsetPosition();

private:
	inline void requestPanel();

private Q_SLOTS:
	/**
	 * \brief Slot Event Functions
	 */
	void changedRobotDescription();
	void changedTaskSolutionTopic();
	void onTasksInserted(const QModelIndex& parent, int first, int last);
	void onTasksRemoved(const QModelIndex& parent, int first, int last);
	void onTaskDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);

	void taskDescriptionCB(const moveit_task_constructor_msgs::TaskDescriptionConstPtr& msg);
	void taskStatisticsCB(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& msg);
	void taskSolutionCB(const moveit_task_constructor_msgs::SolutionConstPtr& msg);

protected:
	ros::Subscriber task_solution_sub;
	ros::Subscriber task_description_sub;
	ros::Subscriber task_statistics_sub;

	// The trajectory playback component
	std::unique_ptr<TaskSolutionVisualization> trajectory_visual_;
	// The TaskListModel storing actual task and solution data
	std::unique_ptr<TaskListModel> task_list_model_;
	bool panel_requested_;

	// Load robot model
	rdf_loader::RDFLoaderPtr rdf_loader_;
	moveit::core::RobotModelConstPtr robot_model_;

	// topic namespace for ROS interfaces of task
	std::string base_ns_;
	// Indicates whether description was received for current task
	bool received_task_description_;

	// Properties
	rviz::StringProperty* robot_description_property_;
	rviz::RosTopicProperty* task_solution_topic_property_;
	rviz::Property* tasks_property_;
};

}  // namespace moveit_rviz_plugin
