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

#include <rviz/panel.h>
#include <moveit/macros/class_forward.h>

namespace moveit_rviz_plugin {

class TaskSolutionVisualization;
MOVEIT_CLASS_FORWARD(TaskListModel)
typedef std::weak_ptr<TaskListModel> TaskListModelWeakPtr;

/** The TaskPanel displays information about manipulation tasks in the system.
 *
 *  Subscribing to task_monitoring and task_solution topics, it collects information
 *  about running tasks and their solutions and allows to inspect both,
 *  successful solutions and failed solution attempts.
 */
class TaskPanelPrivate;
class TaskPanel: public rviz::Panel
{
	Q_OBJECT
	Q_DECLARE_PRIVATE(TaskPanel)
	TaskPanelPrivate *d_ptr;

public:
	TaskPanel(QWidget* parent = 0);
	~TaskPanel();

	void onInitialize() override;
	void load(const rviz::Config& config) override;
	void save(rviz::Config config) const override;

public Q_SLOTS:
	void addTask();
	void showStageDockWidget();

protected Q_SLOTS:
	void removeSelectedStages();
	void onCurrentStageChanged(const QModelIndex &current, const QModelIndex &previous);
	void onCurrentSolutionChanged(const QModelIndex &current, const QModelIndex &previous);
};

}
