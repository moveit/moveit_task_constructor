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

/* Author: Robert Haschke */

#pragma once

#include <utils/tree_merge_proxy_model.h>
#include <moveit/macros/class_forward.h>
#include <QVector>

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(BaseTaskModel);
MOVEIT_CLASS_FORWARD(TaskListModel);
MOVEIT_CLASS_FORWARD(TaskDisplay);

/** MetaTaskListModel maintains a model of multiple registered TaskListModels,
 *  which are grouped in a hierarchical fashion according to the name of the
 *  associated display.
 *
 *  All TaskPanel instances use the singleton instance of this class
 *  to show all tasks known to the system.
 *
 *  This is a singleton instance.
 */
class MetaTaskListModel : public utils::TreeMergeProxyModel
{
	Q_OBJECT

	// 1:1 correspondence of displays to models
	QVector<TaskDisplay*> display_;

	/// class is singleton
	MetaTaskListModel();
	MetaTaskListModel(const MetaTaskListModel&) = delete;
	void operator=(const MetaTaskListModel&) = delete;

	// hide this, as we want to offer another API
	using utils::TreeMergeProxyModel::insertModel;

private Q_SLOTS:
	void onRowsRemoved(const QModelIndex& parent, int first, int last);
	void onDisplayNameChanged(const QString& name);

public:
	static MetaTaskListModel& instance();

	/// insert a new TaskListModel together with it's associated display
	bool insertModel(TaskListModel* model, TaskDisplay* display);

	bool setData(const QModelIndex& index, const QVariant& value, int role) override;
	bool removeRows(int row, int count, const QModelIndex& parent) override;

	/// retrieve TaskListModel and TaskDisplay corresponding to given index
	std::pair<TaskListModel*, TaskDisplay*> getTaskListModel(const QModelIndex& index) const;
	/// retrieve TaskModel and its source index corresponding to given proxy index
	std::pair<BaseTaskModel*, QModelIndex> getTaskModel(const QModelIndex& index) const;
};
}  // namespace moveit_rviz_plugin
