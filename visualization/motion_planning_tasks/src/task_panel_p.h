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

#include "task_panel.h"
#include "ui_task_panel.h"

#include <rviz/panel.h>
#include <rviz/properties/property_tree_model.h>
#include <QPointer>

namespace moveit_rviz_plugin {

class BaseTaskModel;
class TaskListModel;
class TaskDisplay;

class TaskPanelPrivate : public Ui_TaskPanel {
public:
	TaskPanelPrivate(TaskPanel *q_ptr);

	void initSettings(rviz::Property *root);

	/// retrieve TaskListModel corresponding to given index
	inline std::pair<TaskListModel*, TaskDisplay*>
	getTaskListModel(const QModelIndex &index) const;

	/// retrieve TaskModel corresponding to given index
	inline std::pair<BaseTaskModel*, QModelIndex>
	getTaskModel(const QModelIndex& index) const;

	/// unlock locked_display_ if given display is different
	void unlock(TaskDisplay *display);

	TaskPanel* q_ptr;
	rviz::PropertyTreeModel* settings;
	QPointer<TaskDisplay> locked_display_;
	rviz::WindowManagerInterface* window_manager_;

	static QPointer<TaskPanel> global_instance_;
	static uint global_use_count_;
};

}
