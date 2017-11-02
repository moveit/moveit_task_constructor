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

#include "task_list_model.h"
#include "local_task_model.h"
#include "remote_task_model.h"

#include <ros/console.h>

namespace moveit_rviz_plugin {

QString TaskListModel::horizontalHeader(int column)
{
	switch (column) {
	case 0: return tr("Name");
	case 1: return tr("# solved");
	case 2: return tr("# failed");
	}
	return QString();
}

QVariant BaseTaskModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
		return TaskListModel::horizontalHeader(section);
	return QAbstractItemModel::headerData(section, orientation, role);
}

Qt::ItemFlags BaseTaskModel::flags(const QModelIndex &index) const
{
	Qt::ItemFlags flags = QAbstractItemModel::flags(index);
	if (index.column() == 0)
		flags |= Qt::ItemIsEditable; // name is editable
	return flags;
}

enum TaskModelFlag {
	IS_DESTROYED = 0x01,
};

class TaskListModelPrivate {
public:
	Q_DECLARE_PUBLIC(TaskListModel)
	TaskListModel* q_ptr;

	struct BaseModelData {
		BaseModelData(BaseTaskModel* m) : model_(m) {}

		BaseTaskModel* model_;
		// map of proxy=source QModelIndex's internal pointer to source QModelIndex
		QHash<void*, QModelIndex> proxy_to_source_mapping_;
		unsigned int flags_ = 0;
	};

	// top-level items
	std::vector<BaseModelData> tasks_;

	// map from remote task IDs to tasks
	// if task is destroyed remotely, it is marked with flag IS_DESTROYED
	// if task is removed locally from tasks vector, it is marked with a nullptr
	std::map<std::string, RemoteTaskModel*> remote_tasks_;

public:
	TaskListModelPrivate(TaskListModel* q_ptr) : q_ptr(q_ptr) {}

	// retrieve the source_index corresponding to proxy_index
	QModelIndex mapToSource(const QModelIndex &proxy_index, BaseModelData **task = nullptr) const {
		Q_ASSERT(proxy_index.isValid());
		Q_ASSERT(proxy_index.model() == q_ptr);

		void* internal_pointer = proxy_index.internalPointer();
		for (const BaseModelData& t : tasks_) {
			// handling of top-level items
			if (internal_pointer == t.model_) {
				if (task) *task = const_cast<BaseModelData*>(&t);
				return t.model_->index(0, proxy_index.column());
			}
			// for all other levels, internal_pointer maps to source parent
			auto it = t.proxy_to_source_mapping_.constFind(internal_pointer);
			if (it != t.proxy_to_source_mapping_.constEnd()) {
				if (task) *task = const_cast<BaseModelData*>(&t);
				return t.model_->index(proxy_index.row(), proxy_index.column(), *it);
			}
		}
		Q_ASSERT(false);
	}

	QModelIndex mapFromSource(const QModelIndex &src, BaseModelData *task = nullptr) const {
		if (!src.isValid())
			return QModelIndex();

		QModelIndex src_parent = src.parent();
		if (!src_parent.isValid()) { // top-level item
			void* internal_pointer = src.internalPointer();
			int row = 0;
			for (const BaseModelData& t : tasks_) {
				if (t.model_->index(0, 0).internalPointer() == internal_pointer)
					return q_ptr->createIndex(row, src.column(), t.model_);
				++row;
			}
			Q_ASSERT(false);
		}

		// store source index in mapping, currently only if task was provided (coming top-down)
		// TODO: do we need to populate the mapping also when coming bottom-up?
		// This would require climbing up the tree until we reach root
		if (task) task->proxy_to_source_mapping_.insert(src_parent.internalPointer(), src_parent);

		// use internal pointer from parent
		return q_ptr->createIndex(src.row(), src.column(), src_parent.internalPointer());
	}

private:
	void _q_sourceRowsAboutToBeInserted(const QModelIndex &parent, int start, int end);
	void _q_sourceRowsInserted(const QModelIndex &parent, int start, int end);
	void _q_sourceRowsAboutToBeRemoved(const QModelIndex &parent, int start, int end);
	void _q_sourceRowsRemoved(const QModelIndex &parent, int start, int end);
	void _q_sourceRowsAboutToBeMoved(const QModelIndex &sourceParent, int sourceStart, int sourceEnd, const QModelIndex &destParent, int dest);
	void _q_sourceRowsMoved(const QModelIndex &sourceParent, int sourceStart, int sourceEnd, const QModelIndex &destParent, int dest);

	void _q_sourceDataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles);
};


TaskListModel::TaskListModel(QObject *parent)
   : QAbstractItemModel(parent)
{
	d_ptr = new TaskListModelPrivate(this);
	ROS_DEBUG_NAMED("TaskListModel", "created TaskListModel: %p", this);
}

TaskListModel::~TaskListModel() {
	ROS_DEBUG_NAMED("TaskListModel", "destroying TaskListModel: %p", this);
	delete d_ptr;
}

int TaskListModel::rowCount(const QModelIndex &parent) const
{
	Q_D(const TaskListModel);
	if (parent.column() > 0)
		return 0;

	if (!parent.isValid()) // root
		return d->tasks_.size();

	QModelIndex src_parent = d->mapToSource(parent);
	return src_parent.model()->rowCount(src_parent);
}

QModelIndex TaskListModel::index(int row, int column, const QModelIndex &parent) const
{
	Q_D(const TaskListModel);
	if (row < 0 || column < 0 || column >= columnCount(parent))
		return QModelIndex();

	if (!parent.isValid()) { // top-level items
		if ((size_t)row >= d->tasks_.size())
			return QModelIndex();

		TaskListModelPrivate::BaseModelData& task
		      = const_cast<TaskListModelPrivate::BaseModelData&>(d->tasks_.at(row));
		// for top-level item, internal pointer refers to model
		return createIndex(row, column, task.model_);
	}

	// other items need to refer to operation on source model
	TaskListModelPrivate::BaseModelData *task;
	QModelIndex src_parent = d->mapToSource(parent, &task);
	return d->mapFromSource(task->model_->index(row, column, src_parent), task);
}

QModelIndex TaskListModel::parent(const QModelIndex &child) const
{
	Q_D(const TaskListModel);
	if (!child.isValid())
		return QModelIndex();

	TaskListModelPrivate::BaseModelData *task;
	QModelIndex src_parent = d->mapToSource(child, &task).parent();
	return d->mapFromSource(src_parent, task);
}

Qt::ItemFlags TaskListModel::flags(const QModelIndex &index) const
{
	Q_D(const TaskListModel);

	if (!index.isValid())
		return QAbstractItemModel::flags(index);

	QModelIndex src_index = d->mapToSource(index);
	return src_index.model()->flags(src_index);
}

QVariant TaskListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
		return TaskListModel::horizontalHeader(section);
	else
		return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant TaskListModel::data(const QModelIndex &index, int role) const
{
	Q_D(const TaskListModel);
	QModelIndex src_index = d->mapToSource(index);
	return src_index.model()->data(src_index, role);
}

bool TaskListModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	Q_D(const TaskListModel);
	TaskListModelPrivate::BaseModelData *task;
	QModelIndex src_index = d->mapToSource(index, &task);
	Q_ASSERT(task->model_ == src_index.model());
	return task->model_->setData(src_index, value, role);
}

bool TaskListModel::removeRows(int row, int count, const QModelIndex &parent)
{
	// TODO
	return false;
}

// process a task monitoring message:
// update existing RemoteTask, create a new one,
// or (if msg.stages is empty) delete an existing one
void TaskListModel::processTaskMessage(const moveit_task_constructor::Task &msg)
{
	Q_D(TaskListModel);

	// retrieve existing or insert new remote task for given id
	auto it_inserted = d->remote_tasks_.insert(std::make_pair(msg.id, nullptr));
	bool created = it_inserted.second;
	RemoteTaskModel*& remote_task = it_inserted.first->second;

	// empty stages list indicates, that this remote task is not available anymore
	if (msg.stages.empty()) {
		if (!remote_task) { // task was already deleted locally
			// we can now remove it from remote_tasks_
			d->remote_tasks_.erase(it_inserted.first);
			return;
		}
		// task is still in use, mark it as destroyed
		for (auto& t : d->tasks_) {
			if (t.model_ == remote_task) {
				t.flags_ |= IS_DESTROYED;
				break;
			}
		}
	} else if (created) { // create new task model, if ID was not known before
		// the model is managed by this instance via Qt's parent-child mechanism
		remote_task = new RemoteTaskModel(this);
	}
	if (!remote_task)
		return; // task is not in use anymore

	remote_task->processTaskMessage(msg);

	// insert newly created model into this' model instance
	if (created) {
		ROS_DEBUG_NAMED("TaskListModel", "received new Task: %s", msg.id.c_str());
		insertTask(remote_task, -1);
	}
}

void TaskListModel::processSolutionMessage(const moveit_task_constructor::Solution &msg)
{
	// TODO
}

BaseTaskModel *TaskListModel::getTask(int row) const
{
	Q_D(const TaskListModel);
	if (row < 0 || (size_t)row >= d->tasks_.size())
		return nullptr;
	return d->tasks_.at(row).model_;
}

void TaskListModel::insertTask(BaseTaskModel* model, int row) {
	Q_D(TaskListModel);

	if (row < 0 || (size_t)row > d->tasks_.size())
		row = d->tasks_.size();

	auto it = d->tasks_.begin();
	std::advance(it, row);

	ROS_DEBUG_NAMED("TaskListModel", "%p: inserting task: %p", this, model);
	beginInsertRows(QModelIndex(), row, row);
	d->tasks_.insert(it, TaskListModelPrivate::BaseModelData(model));
	endInsertRows();

	// notice destruction of task
	if (model->parent() != this)
		connect(model, &BaseTaskModel::destroyed,
		        [this](QObject* o) { removeTask(static_cast<BaseTaskModel*>(o), false); });

	connect(model, SIGNAL(rowsAboutToBeInserted(QModelIndex,int,int)),
	        this, SLOT(_q_sourceRowsAboutToBeInserted(QModelIndex,int,int)));
	connect(model, SIGNAL(rowsInserted(QModelIndex,int,int)),
	        this, SLOT(_q_sourceRowsInserted(QModelIndex,int,int)));
	connect(model, SIGNAL(rowsAboutToBeRemoved(QModelIndex,int,int)),
	        this, SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex,int,int)));
	connect(model, SIGNAL(rowsRemoved(QModelIndex,int,int)),
	        this, SLOT(_q_sourceRowsRemoved(QModelIndex,int,int)));
	connect(model, SIGNAL(rowsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int)),
	        this, SLOT(_q_sourceRowsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int)));
	connect(model, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)),
	        this, SLOT(_q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int)));
	connect(model, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
	        this, SLOT(_q_sourceDataChanged(QModelIndex,QModelIndex,QVector<int>)));
}

bool TaskListModel::removeTask(BaseTaskModel* model, bool disconnect_signals) {
	Q_D(TaskListModel);

	// find row corresponding to model
	auto it = std::find_if(d->tasks_.begin(), d->tasks_.end(),
	                       [model](const TaskListModelPrivate::BaseModelData& data) { return	data.model_ == model; });
	if (it == d->tasks_.end())
		return false; // model not found

	ROS_DEBUG_NAMED("TaskListModel", "%p: removing task: %p", this, model);
	size_t row = it - d->tasks_.begin();
	beginRemoveRows(QModelIndex(), row, row);
	d->tasks_.insert(it, TaskListModelPrivate::BaseModelData(model));
	endRemoveRows();

	if (disconnect_signals) { // reacting on signal destroyed(), signals are already disconnected
		disconnect(model, SIGNAL(rowsAboutToBeInserted(QModelIndex,int,int)),
		           this, SLOT(_q_sourceRowsAboutToBeInserted(QModelIndex,int,int)));
		disconnect(model, SIGNAL(rowsInserted(QModelIndex,int,int)),
		           this, SLOT(_q_sourceRowsInserted(QModelIndex,int,int)));
		disconnect(model, SIGNAL(rowsAboutToBeRemoved(QModelIndex,int,int)),
		           this, SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex,int,int)));
		disconnect(model, SIGNAL(rowsRemoved(QModelIndex,int,int)),
		           this, SLOT(_q_sourceRowsRemoved(QModelIndex,int,int)));
		disconnect(model, SIGNAL(rowsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int)),
		           this, SLOT(_q_sourceRowsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int)));
		disconnect(model, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)),
		           this, SLOT(_q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int)));
		disconnect(model, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
		           this, SLOT(_q_sourceDataChanged(QModelIndex,QModelIndex,QVector<int>)));
	}
	return true;
}

void TaskListModelPrivate::_q_sourceRowsAboutToBeInserted(const QModelIndex &parent, int start, int end)
{
	q_ptr->beginInsertRows(mapFromSource(parent), start, end);
}

void TaskListModelPrivate::_q_sourceRowsAboutToBeMoved(const QModelIndex &sourceParent, int sourceStart, int sourceEnd, const QModelIndex &destParent, int dest)
{
	q_ptr->beginMoveRows(mapFromSource(sourceParent), sourceStart, sourceEnd, mapFromSource(destParent), dest);
}

void TaskListModelPrivate::_q_sourceRowsAboutToBeRemoved(const QModelIndex &parent, int start, int end)
{
	q_ptr->beginRemoveRows(mapFromSource(parent), start, end);
}

void TaskListModelPrivate::_q_sourceRowsInserted(const QModelIndex &parent, int start, int end)
{
	Q_UNUSED(parent)
	Q_UNUSED(start)
	Q_UNUSED(end)
	q_ptr->endInsertRows();
}

void TaskListModelPrivate::_q_sourceRowsMoved(const QModelIndex &sourceParent, int sourceStart, int sourceEnd, const QModelIndex &destParent, int dest)
{
	Q_UNUSED(sourceParent)
	Q_UNUSED(sourceStart)
	Q_UNUSED(sourceEnd)
	Q_UNUSED(destParent)
	Q_UNUSED(dest)
	q_ptr->endMoveRows();
}

void TaskListModelPrivate::_q_sourceRowsRemoved(const QModelIndex &parent, int start, int end)
{
	Q_UNUSED(parent)
	Q_UNUSED(start)
	Q_UNUSED(end)
	q_ptr->endRemoveRows();
}

void TaskListModelPrivate::_q_sourceDataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
	q_ptr->dataChanged(mapFromSource(topLeft), mapFromSource(bottomRight), roles);
}

}

#include "moc_task_list_model.cpp"
