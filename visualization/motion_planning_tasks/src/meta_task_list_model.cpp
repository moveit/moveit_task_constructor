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

#include "meta_task_list_model.h"
#include "task_list_model.h"
#include "task_display.h"

namespace moveit_rviz_plugin {

MetaTaskListModel::MetaTaskListModel() {
	connect(this, SIGNAL(rowsRemoved(QModelIndex, int, int)), this, SLOT(onRowsRemoved(QModelIndex, int, int)));
}

MetaTaskListModel& MetaTaskListModel::instance() {
	static MetaTaskListModel instance;
	return instance;
}

bool MetaTaskListModel::insertModel(TaskListModel* model, TaskDisplay* display) {
	if (!model || !display)
		return false;
	if (display_.contains(display))
		return false;
	if (!TreeMergeProxyModel::insertModel(display->getName(), model))
		return false;

	// keep display name in sync with model name
	display_.push_back(display);
	connect(display, SIGNAL(objectNameChanged(QString)), this, SLOT(onDisplayNameChanged(QString)));
	return true;
}

void MetaTaskListModel::onRowsRemoved(const QModelIndex& parent, int first, int last) {
	if (!parent.isValid()) {
		display_.remove(first, last - first + 1);
	}
}

void MetaTaskListModel::onDisplayNameChanged(const QString& name) {
	int row = display_.indexOf(static_cast<TaskDisplay*>(sender()));
	if (row < 0)
		return;

	QModelIndex idx = index(row, 0);
	if (idx.data() == name)
		return;

	setData(idx, name, Qt::EditRole);
}

bool MetaTaskListModel::setData(const QModelIndex& index, const QVariant& value, int role) {
	bool result = TreeMergeProxyModel::setData(index, value, role);
	if (result && isGroupItem(index)) {
		display_.at(index.row())->setName(value.toString());
	}
	return result;
}

bool MetaTaskListModel::removeRows(int row, int count, const QModelIndex& parent) {
	if (!parent.isValid())  // forbid removal of top-level items (displays)
		return false;
	return TreeMergeProxyModel::removeRows(row, count, parent);
}

std::pair<TaskListModel*, TaskDisplay*> MetaTaskListModel::getTaskListModel(const QModelIndex& index) const {
	QAbstractItemModel* m = getModel(index).first;
	if (!m)
		return std::make_pair(nullptr, nullptr);

	Q_ASSERT(dynamic_cast<TaskListModel*>(m));
	return std::make_pair(static_cast<TaskListModel*>(m), display_.at(getRow(m)));
}

std::pair<BaseTaskModel*, QModelIndex> MetaTaskListModel::getTaskModel(const QModelIndex& index) const {
	if (!index.isValid())
		return std::make_pair(nullptr, QModelIndex());

	auto result = getModel(index);
	Q_ASSERT(result.first && dynamic_cast<TaskListModel*>(result.first));

	if (!result.second.isValid())
		return std::make_pair(nullptr, QModelIndex());

	auto m = static_cast<TaskListModel*>(result.first)->getModel(result.second);
	Q_ASSERT(dynamic_cast<BaseTaskModel*>(m.first));
	return std::make_pair(static_cast<BaseTaskModel*>(m.first), m.second);
}
}  // namespace moveit_rviz_plugin
