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
#include "factory_model.h"

#include <ros/console.h>
#include <QMimeData>
#include <qevent.h>

namespace moveit_rviz_plugin {

static const std::string LOGNAME("TaskListModel");

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


StageFactoryPtr getStageFactory()
{
	static std::weak_ptr<StageFactory> factory;
	if (!factory.expired())
		return factory.lock();

	try {
		StageFactoryPtr result(new StageFactory("moveit_task_constructor_core",
		                                        "moveit::task_constructor::Stage"));
		// Hm. pluglinlib / ClassLoader cannot instantiate classes in implicitly loaded libs
		result->addBuiltInClass<moveit::task_constructor::SerialContainer>("Serial Container", "");
		factory = result; // remember for future uses
		return result;
	} catch (const std::exception &e) {
		ROS_ERROR("Failed to initialize StageFactory");
		return StageFactoryPtr();
	}
}


void TaskListModel::onRemoveModel(QAbstractItemModel *model)
{
	FlatMergeProxyModel::onRemoveModel(model);
	if (model->parent() == this)
		model->deleteLater();
}

TaskListModel::TaskListModel(QObject *parent)
   : FlatMergeProxyModel(parent)
{
	ROS_DEBUG_NAMED(LOGNAME, "created TaskListModel: %p", this);
}

TaskListModel::~TaskListModel() {
	ROS_DEBUG_NAMED(LOGNAME, "destroying TaskListModel: %p", this);
}

void TaskListModel::setScene(const planning_scene::PlanningSceneConstPtr &scene)
{
	scene_ = scene;
}

void TaskListModel::setStageFactory(const StageFactoryPtr &factory)
{
	stage_factory_ = factory;
}

// re-implemented from base class to allow dropping
Qt::ItemFlags TaskListModel::flags(const QModelIndex &index) const
{
	if (!index.isValid()) {
		Qt::ItemFlags f = QAbstractItemModel::flags(index);
		// dropping at root will create a new local task
		if (stage_factory_)
			f |= Qt::ItemIsDropEnabled;
		return f;
	}

	return FlatMergeProxyModel::flags(index);
}

QStringList TaskListModel::mimeTypes() const
{
	QStringList result;
	if (stage_factory_)
		result << stage_factory_->mimeType();
	return result;
}

QVariant TaskListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
		return TaskListModel::horizontalHeader(section);
	else
		return QAbstractItemModel::headerData(section, orientation, role);
}

// process a task description message:
// update existing RemoteTask, create a new one, or (if msg.stages is empty) delete an existing one
void TaskListModel::processTaskDescriptionMessage(const std::string& id,
                                                  const moveit_task_constructor_msgs::TaskDescription &msg)
{
	// retrieve existing or insert new remote task for given id
	auto it_inserted = remote_tasks_.insert(std::make_pair(id, nullptr));
	bool created = it_inserted.second;
	RemoteTaskModel*& remote_task = it_inserted.first->second;

	if (!msg.stages.empty() && remote_task && remote_task->taskFlags() & BaseTaskModel::IS_DESTROYED)
		created = true; // re-create remote task after it was destroyed beforehand

	// empty list indicates, that this remote task is not available anymore
	if (msg.stages.empty()) {
		if (!remote_task) { // task was already deleted locally
			// we can now remove it from remote_tasks_
			remote_tasks_.erase(it_inserted.first);
			return;
		}
	} else if (created) { // create new task model, if ID was not known before
		// the model is managed by this instance via Qt's parent-child mechanism
		remote_task = new RemoteTaskModel(scene_, this);
	}
	if (!remote_task)
		return; // task is not in use anymore

	remote_task->processStageDescriptions(msg.stages);

	// insert newly created model into this' model instance
	if (created) {
		ROS_DEBUG_NAMED(LOGNAME, "received new task: %s", msg.id.c_str());
		insertModel(remote_task, -1);
	}
}

// process a task statistics message
void TaskListModel::processTaskStatisticsMessage(const std::string &id,
                                                 const moveit_task_constructor_msgs::TaskStatistics &msg)
{
	auto it = remote_tasks_.find(id);
	if (it == remote_tasks_.cend())
		return; // unkown task

	RemoteTaskModel* remote_task = it->second;
	if (!remote_task)
		return; // task is not in use anymore

	remote_task->processStageStatistics(msg.stages);
}

DisplaySolutionPtr TaskListModel::processSolutionMessage(const std::string &id,
                                                         const moveit_task_constructor_msgs::Solution &msg)
{
	auto it = remote_tasks_.find(id);
	if (it == remote_tasks_.cend())
		return DisplaySolutionPtr(); // unkown task

	RemoteTaskModel* remote_task = it->second;
	if (!remote_task)
		return DisplaySolutionPtr(); // task is not in use anymore

	return remote_task->processSolutionMessage(msg);
}

bool TaskListModel::dropMimeData(const QMimeData *mime, Qt::DropAction action, int row, int column, const QModelIndex &parent)
{
	Q_UNUSED(column);
	if (!stage_factory_)
		return false;
	const QString& mime_type = stage_factory_->mimeType();
	if (!mime->hasFormat(mime_type))
		return false;

	if (!parent.isValid() && mime->hasFormat(mime_type)) {
		QString error;
		moveit::task_constructor::Stage* stage
		      = stage_factory_->makeRaw(mime->data(mime_type), &error);
		std::unique_ptr<moveit::task_constructor::ContainerBase> container
		      (dynamic_cast<moveit::task_constructor::ContainerBase*>(stage));
		if (!container) { // only accept container at root level
			if (stage) delete stage;
			return false;
		}

		// create a new local task using the given container as root
		insertModel(new LocalTaskModel(std::move(container), this), row);
		return true;
	}

	return FlatMergeProxyModel::dropMimeData(mime, action, row, column, parent);
}

Qt::DropActions TaskListModel::supportedDropActions() const
{
	return Qt::CopyAction | Qt::MoveAction;
}


TaskListView::TaskListView(QWidget *parent)
   : QTreeView(parent)
{
}

// dropping onto an item, should expand this item
void TaskListView::dropEvent(QDropEvent *event)
{
	QModelIndex index = indexAt(event->pos());
	QTreeView::dropEvent(event);
	if (event->isAccepted())
		expand(index);
}

}

#include "moc_task_list_model.cpp"
