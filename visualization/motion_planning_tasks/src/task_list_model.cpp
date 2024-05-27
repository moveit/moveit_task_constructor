/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2020, Hamburg University
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
#include "task_panel.h"
#include "factory_model.h"
#include "icons.h"

#include <QMimeData>
#include <QHeaderView>
#include <QScrollBar>
#include <qevent.h>
#include <numeric>

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.task_list_model");

QVariant TaskListModel::horizontalHeader(int column, int role) {
	switch (role) {
		case Qt::DisplayRole:
			switch (column) {
				case 0:
					return tr("name");
				case 1:
					return tr(u8"✓");
				case 2:
					return tr(u8"✗");
				case 3:
					return tr("time");
			}
			break;

		case Qt::ForegroundRole:
			switch (column) {
				case 1:
					return QColor(Qt::darkGreen);
				case 2:
					return QColor(Qt::red);
			}
			break;

		case Qt::TextAlignmentRole:
			return Qt::AlignLeft;

		case Qt::ToolTipRole:
			switch (column) {
				case 1:
					return tr("successful solutions");
				case 2:
					return tr("failed solution attempts");
				case 3:
					return tr("total computation time [s]");
				case 4:
					return tr("pending");
			}
			break;
	}

	return QVariant();
}

QVariant BaseTaskModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Horizontal)
		return TaskListModel::horizontalHeader(section, role);
	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant BaseTaskModel::data(const QModelIndex& index, int role) const {
	switch (role) {
		case Qt::TextAlignmentRole:
			return index.column() == 0 ? Qt::AlignLeft : Qt::AlignRight;
	}
	return QVariant();
}

QVariant BaseTaskModel::flowIcon(moveit::task_constructor::InterfaceFlags f) {
	static const QIcon CONNECT_ICON = icons::CONNECT.icon();
	static const QIcon FORWARD_ICON = icons::FORWARD.icon();
	static const QIcon BACKWARD_ICON = icons::BACKWARD.icon();
	static const QIcon GENERATE_ICON = icons::GENERATE.icon();

	if (f == InterfaceFlags(CONNECT))
		return CONNECT_ICON;
	if (f == InterfaceFlags(PROPAGATE_FORWARDS))
		return FORWARD_ICON;
	if (f == InterfaceFlags(PROPAGATE_BACKWARDS))
		return BACKWARD_ICON;
	if (f == InterfaceFlags(GENERATE))
		return GENERATE_ICON;

	return QVariant();
}

StageFactoryPtr getStageFactory() {
	static std::weak_ptr<StageFactory> factory;
	StageFactoryPtr result = factory.lock();
	if (result)
		return result;

	try {
		result.reset(new StageFactory("moveit_task_constructor_core", "moveit::task_constructor::Stage"));
		// Hm. pluglinlib / ClassLoader cannot instantiate classes in implicitly loaded libs
		result->addBuiltInClass<moveit::task_constructor::SerialContainer>("Serial Container", "");
		factory = result;  // remember for future uses
		return result;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(LOGGER, "Failed to initialize StageFactory");
		return StageFactoryPtr();
	}
}

void TaskListModel::onRemoveModel(QAbstractItemModel* model) {
	FlatMergeProxyModel::onRemoveModel(model);
	if (model->parent() == this)
		model->deleteLater();

	// mark model as removed
	auto it = std::find_if(remote_tasks_.begin(), remote_tasks_.end(),
	                       [model](const auto& pair) { return pair.second == model; });
	if (it != remote_tasks_.end())
		it->second = nullptr;
}

TaskListModel::TaskListModel(QObject* parent)
  : FlatMergeProxyModel(parent), old_task_handling_(TaskView::OLD_TASK_REPLACE) {
	RCLCPP_DEBUG(LOGGER, "created TaskListModel: %p", this);
	setStageFactory(getStageFactory());
}

TaskListModel::~TaskListModel() {
	RCLCPP_DEBUG(LOGGER, "destroying TaskListModel: %p", this);
	// inform MetaTaskListModel that we will remove our stuff
	removeRows(0, rowCount());
	// free RemoteTaskModels
	for (auto& pair : remote_tasks_)
		delete pair.second;
}

void TaskListModel::setScene(const planning_scene::PlanningSceneConstPtr& scene) {
	scene_ = scene;
}

void TaskListModel::setDisplayContext(rviz_common::DisplayContext* display_context) {
	display_context_ = display_context;
}

void TaskListModel::setStageFactory(const StageFactoryPtr& factory) {
	stage_factory_ = factory;
	if (stage_factory_)
		setMimeTypes({ stage_factory_->mimeType() });
}

// re-implemented from base class to allow dropping
Qt::ItemFlags TaskListModel::flags(const QModelIndex& index) const {
	auto f = FlatMergeProxyModel::flags(index);

	if (!index.isValid()) {
		// dropping at root will create a new local task
		if (stage_factory_)
			f |= Qt::ItemIsDropEnabled;
	}
	return f;
}

void TaskListModel::setOldTaskHandling(int mode) {
	old_task_handling_ = mode;
}

void TaskListModel::highlightStage(size_t id) {
	if (!active_task_model_)
		return;
	QModelIndex old_index = highlighted_row_index_;
	QModelIndex new_index = active_task_model_->indexFromStageId(id);
	if (new_index.isValid())
		highlighted_row_index_ = new_index = mapFromSource(new_index);
	else
		highlighted_row_index_ = QModelIndex();

	if (old_index == new_index)
		return;
	if (old_index.isValid())
		Q_EMIT dataChanged(old_index, old_index.sibling(old_index.row(), columnCount() - 1));
	if (new_index.isValid())
		Q_EMIT dataChanged(new_index, new_index.sibling(new_index.row(), columnCount() - 1));
}

QVariant TaskListModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Horizontal)
		return TaskListModel::horizontalHeader(section, role);
	else
		return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant TaskListModel::data(const QModelIndex& index, int role) const {
	if (role == Qt::BackgroundRole && index.isValid() && index.row() == highlighted_row_index_.row() &&
	    index.parent() == highlighted_row_index_.parent())
		return QColor(Qt::yellow);
	return FlatMergeProxyModel::data(index, role);
}

// process a task description message:
// update existing RemoteTask, create a new one, or (if msg.stages is empty) delete an existing one
void TaskListModel::processTaskDescriptionMessage(const moveit_task_constructor_msgs::msg::TaskDescription& msg,
                                                  const std::string& service_name) {
	// retrieve existing or insert new remote task for given task id
	auto it_inserted = remote_tasks_.insert(std::make_pair(msg.task_id, nullptr));
	const auto& task_it = it_inserted.first;
	RemoteTaskModel*& remote_task = task_it->second;

	if (!msg.stages.empty() && remote_task && (remote_task->taskFlags() & BaseTaskModel::IS_DESTROYED)) {
		// task overriding previous one that was already marked destroyed, but not yet removed from model
		if (old_task_handling_ != TaskView::OLD_TASK_KEEP)
			removeModel(remote_task);
		remote_task = nullptr;  // resetting to nullptr will trigger creation of a new task
	}

	// empty list indicates, that this remote task was destroyed and we won't get updates for it
	if (msg.stages.empty()) {
		// always remove destroyed RemoteTask?
		if (old_task_handling_ == TaskView::OLD_TASK_REMOVE && remote_task) {
			removeModel(remote_task);
			remote_tasks_.erase(task_it);
			return;
		}
		if (remote_task)  // keep the task, but mark it as destroyed
			remote_task->processStageDescriptions(msg.stages);
	} else if (!remote_task) {  // create new task model, if ID was not known before
		// the model is managed by this instance via Qt's parent-child mechanism
		remote_task = new RemoteTaskModel(service_name, scene_, display_context_, this);
		remote_task->processStageDescriptions(msg.stages);
		RCLCPP_DEBUG(LOGGER, "received new task: %s (%s)", msg.stages[0].name.c_str(), msg.task_id.c_str());
		// insert newly created model into this' model instance
		insertModel(remote_task, -1);

		// HACK: always use the last created model as active
		active_task_model_ = remote_task;
	} else  // normal update
		remote_task->processStageDescriptions(msg.stages);
}

// process a task statistics message
void TaskListModel::processTaskStatisticsMessage(const moveit_task_constructor_msgs::msg::TaskStatistics& msg) {
	auto it = remote_tasks_.find(msg.task_id);
	if (it == remote_tasks_.cend()) {
		RCLCPP_WARN(LOGGER, "unknown task: %s", msg.task_id.c_str());
		return;
	}

	RemoteTaskModel* remote_task = it->second;
	if (!remote_task || (remote_task->taskFlags() & RemoteTaskModel::IS_DESTROYED))
		return;  // task is not in use anymore

	remote_task->processStageStatistics(msg.stages);
}

DisplaySolutionPtr TaskListModel::processSolutionMessage(const moveit_task_constructor_msgs::msg::Solution& msg) {
	auto it = remote_tasks_.find(msg.task_id);
	if (it == remote_tasks_.cend())
		return DisplaySolutionPtr();  // unknown task

	RemoteTaskModel* remote_task = it->second;
	if (!remote_task)
		return DisplaySolutionPtr();  // task is not in use anymore

	return remote_task->processSolutionMessage(msg);
}

bool TaskListModel::insertModel(BaseTaskModel* model, int pos) {
	Q_ASSERT(model && model->columnCount() == columnCount());
	// pass on stage factory
	model->setStageFactory(stage_factory_);
	// forward to base class method
	return FlatMergeProxyModel::insertModel(model, pos);
}

BaseTaskModel* TaskListModel::createLocalTaskModel() {
	return new LocalTaskModel(std::make_unique<SerialContainer>("task pipeline"), scene_, display_context_, this);
}

bool TaskListModel::dropMimeData(const QMimeData* mime, Qt::DropAction action, int row, int column,
                                 const QModelIndex& parent) {
	Q_UNUSED(column);
	if (!stage_factory_)
		return false;
	const QString& mime_type = stage_factory_->mimeType();
	if (!mime->hasFormat(mime_type))
		return false;

	if (!parent.isValid() && mime->hasFormat(mime_type)) {
		QString error;
		moveit::task_constructor::Stage* stage = stage_factory_->makeRaw(mime->data(mime_type), &error);
		std::unique_ptr<moveit::task_constructor::ContainerBase> container(
		    dynamic_cast<moveit::task_constructor::ContainerBase*>(stage));
		if (!container) {  // only accept container at root level
			delete stage;
			return false;
		}

		// create a new local task using the given container as root
		auto* m = new LocalTaskModel(std::move(container), scene_, display_context_, this);
		insertModel(m, row);
		return true;
	}

	return FlatMergeProxyModel::dropMimeData(mime, action, row, column, parent);
}

Qt::DropActions TaskListModel::supportedDropActions() const {
	return Qt::CopyAction | Qt::MoveAction;
}

TaskListView::TaskListView(QWidget* parent) : QTreeView(parent) {}

// dropping onto an item, should expand this item
void TaskListView::dropEvent(QDropEvent* event) {
	QModelIndex index = indexAt(event->pos());
	QTreeView::dropEvent(event);
	if (event->isAccepted())
		expand(index);
}

void TaskListView::setModel(QAbstractItemModel* model) {
	QTreeView::setModel(model);
	if (header()->count() >= 4) {
		header()->setSectionResizeMode(0, QHeaderView::Stretch);
		updateColumnWidth();
	}
}

void TaskListView::dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight, const QVector<int>& roles) {
	if (bottomRight.column() > 0) {
		updateColumnWidth();
	}
	QTreeView::dataChanged(topLeft, bottomRight, roles);
}

void TaskListView::updateColumnWidth() {
	for (int i = 3; i > 0; --i) {
		header()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}
}

SolutionListView::SolutionListView(QWidget* parent) : QTreeView(parent) {}

void SolutionListView::setModel(QAbstractItemModel* model) {
	QTreeView::setModel(model);
	updateColumnWidth();
}

void SolutionListView::dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
                                   const QVector<int>& roles) {
	if (bottomRight.column() > 0) {
		updateColumnWidth();
	}
	QTreeView::dataChanged(topLeft, bottomRight, roles);
}

void SolutionListView::resizeEvent(QResizeEvent* e) {
	QTreeView::resizeEvent(e);
	updateColumnWidth();
}

void SolutionListView::updateColumnWidth() {
	// do nothing if current model is not what we expect
	if (header()->count() < 3) {
		return;
	}

	for (int i = 0; i < 2; ++i) {
		header()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
	}

	const int stretch_size = viewport()->size().width() - header()->sectionPosition(2);

	const int content_size = sizeHintForColumn(2);

	header()->resizeSection(2, std::max(stretch_size, content_size));
}

}  // namespace moveit_rviz_plugin

#include "moc_task_list_model.cpp"
