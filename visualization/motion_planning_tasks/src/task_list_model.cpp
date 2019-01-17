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
#include "icons.h"

#include <ros/console.h>
#include <ros/service_client.h>

#include <QMimeData>
#include <QHeaderView>
#include <QScrollBar>
#include <qevent.h>
#include <numeric>

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

static const std::string LOGNAME("TaskListModel");

QVariant TaskListModel::horizontalHeader(int column, int role)
{
	switch (role) {
	case Qt::DisplayRole:
		switch (column) {
		case 0: return tr("Name");
		case 1: return tr(u8"✓");
		case 2: return tr(u8"✗");
		}
		break;

	case Qt::ForegroundRole:
		switch (column) {
		case 1: return QColor(Qt::darkGreen);
		case 2: return QColor(Qt::red);
		}
		break;

	case Qt::TextAlignmentRole:
		return column == 0 ? Qt::AlignLeft : Qt::AlignRight;

	case Qt::ToolTipRole:
		switch (column) {
		case 1: return tr("successful solutions");
		case 2: return tr("failed solution attempts");
		case 3: return tr("pending");
		}
		break;
	}

	return QVariant();
}

QVariant BaseTaskModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal)
		return TaskListModel::horizontalHeader(section, role);
	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant BaseTaskModel::data(const QModelIndex &index, int role) const
{
	switch (role) {
	case Qt::TextAlignmentRole:
		return index.column() == 0 ? Qt::AlignLeft : Qt::AlignRight;
	}
	return QVariant();
}

Qt::ItemFlags BaseTaskModel::flags(const QModelIndex &index) const
{
	Qt::ItemFlags flags = QAbstractItemModel::flags(index);
	if (index.column() == 0)
		flags |= Qt::ItemIsEditable; // name is editable
	return flags;
}

QVariant BaseTaskModel::flowIcon(moveit::task_constructor::InterfaceFlags f)
{
	static const QIcon CONNECT_ICON = icons::CONNECT.icon();
	static const QIcon FORWARD_ICON = icons::FORWARD.icon();
	static const QIcon BACKWARD_ICON = icons::BACKWARD.icon();
	static const QIcon BOTHWAY_ICON = icons::BOTHWAY.icon();
	static const QIcon GENERATE_ICON = icons::GENERATE.icon();

	if (f == InterfaceFlags(CONNECT)) return CONNECT_ICON;
	if (f == InterfaceFlags(PROPAGATE_FORWARDS)) return FORWARD_ICON;
	if (f == InterfaceFlags(PROPAGATE_BACKWARDS)) return BACKWARD_ICON;
	if (f == PROPAGATE_BOTHWAYS) return BOTHWAY_ICON;
	if (f == InterfaceFlags(GENERATE)) return GENERATE_ICON;

	return QVariant();
}


StageFactoryPtr getStageFactory()
{
	static std::weak_ptr<StageFactory> factory;
	StageFactoryPtr result = factory.lock();
	if (result)
		return result;

	try {
		result.reset(new StageFactory("moveit_task_constructor_core",
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

	// mark model as removed
	auto it = std::find_if(remote_tasks_.begin(), remote_tasks_.end(),
	                       [model](const auto& pair) { return pair.second == model; });
	if (it != remote_tasks_.end())
		it->second = nullptr;
}

TaskListModel::TaskListModel(QObject *parent)
   : FlatMergeProxyModel(parent)
{
	ROS_DEBUG_NAMED(LOGNAME, "created TaskListModel: %p", this);
	setStageFactory(getStageFactory());
}

TaskListModel::~TaskListModel() {
	ROS_DEBUG_NAMED(LOGNAME, "destroying TaskListModel: %p", this);
	// inform MetaTaskListModel that we will remove our stuff
	removeRows(0, rowCount());
	// free RemoteTaskModels
	for (auto& pair : remote_tasks_) {
		if (pair.second)
			delete pair.second;
	}
}

void TaskListModel::setScene(const planning_scene::PlanningSceneConstPtr &scene)
{
	scene_ = scene;
}

void TaskListModel::setDisplayContext(rviz::DisplayContext *display_context)
{
	display_context_ = display_context;
}

void TaskListModel::setSolutionClient(ros::ServiceClient *client)
{
	get_solution_client_ = client;
}

void TaskListModel::setStageFactory(const StageFactoryPtr &factory)
{
	stage_factory_ = factory;
	if (stage_factory_)
		setMimeTypes({ stage_factory_->mimeType() });
}

// re-implemented from base class to allow dropping
Qt::ItemFlags TaskListModel::flags(const QModelIndex &index) const
{
	auto f = FlatMergeProxyModel::flags(index);

	if (!index.isValid()) {
		// dropping at root will create a new local task
		if (stage_factory_)
			f |= Qt::ItemIsDropEnabled;
	}
	return f;
}

void TaskListModel::highlightStage(size_t id)
{
	if (!active_task_model_) return;
	QModelIndex old_index = highlighted_row_index_;
	QModelIndex new_index = active_task_model_->indexFromStageId(id);
	if (new_index.isValid())
		highlighted_row_index_ = new_index = mapFromSource(new_index);
	else
		highlighted_row_index_ = QModelIndex();

	if (old_index == new_index) return;
	if (old_index.isValid())
		Q_EMIT dataChanged(old_index, old_index.sibling(old_index.row(), columnCount()-1));
	if (new_index.isValid())
		Q_EMIT dataChanged(new_index, new_index.sibling(new_index.row(), columnCount()-1));
}

QVariant TaskListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal)
		return TaskListModel::horizontalHeader(section, role);
	else
		return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant TaskListModel::data(const QModelIndex &index, int role) const
{
	if (role == Qt::BackgroundRole && index.isValid() &&
	    index.row() == highlighted_row_index_.row() &&
	    index.parent() == highlighted_row_index_.parent())
		return QColor(Qt::yellow);
	return FlatMergeProxyModel::data(index, role);
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

	if (!msg.stages.empty() && remote_task && (remote_task->taskFlags() & BaseTaskModel::IS_DESTROYED)) {
		removeModel(remote_task);
		created = true; // re-create remote task after it was destroyed beforehand
	}

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
		remote_task->setSolutionClient(get_solution_client_);

		// HACK: always use the last created model as active
		active_task_model_ = remote_task;
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
	if (it == remote_tasks_.cend()) {
		ROS_WARN("unknown task: %s", id.c_str());
		return;
	}

	RemoteTaskModel* remote_task = it->second;
	if (!remote_task || (remote_task->taskFlags() & RemoteTaskModel::IS_DESTROYED))
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

bool TaskListModel::insertModel(BaseTaskModel *model, int pos) {
	Q_ASSERT(model && model->columnCount() == columnCount());
	// pass on stage factory
	model->setStageFactory(stage_factory_);
	// forward to base class method
	return FlatMergeProxyModel::insertModel(model, pos);
}

BaseTaskModel* TaskListModel::createLocalTaskModel() {
	return new LocalTaskModel(std::make_unique<SerialContainer>("task pipeline"), scene_, display_context_, this);
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
		auto *m = new LocalTaskModel(std::move(container), scene_, display_context_, this);
		insertModel(m, row);
		return true;
	}

	return FlatMergeProxyModel::dropMimeData(mime, action, row, column, parent);
}

Qt::DropActions TaskListModel::supportedDropActions() const
{
	return Qt::CopyAction | Qt::MoveAction;
}


AutoAdjustingTreeView::AutoAdjustingTreeView(QWidget *parent)
   : QTreeView(parent)
{
	// consider viewportSizeHint()
#if QT_VERSION >= QT_VERSION_CHECK(5, 2, 0)
	setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
#endif
}

void AutoAdjustingTreeView::setStretchSection(int section)
{
	stretch_section_ = section;
	updateGeometry();
}

void AutoAdjustingTreeView::setAutoHideSections(const QList<int> &sections)
{
	auto_hide_cols_ = sections;
	updateGeometry();
}

void AutoAdjustingTreeView::setModel(QAbstractItemModel *model)
{
	size_hints_.clear();
	QTreeView::setModel(model);

	updateGeometry();
}

QSize AutoAdjustingTreeView::viewportSizeHint() const
{
	bool preferred = sizePolicy().horizontalPolicy() & QSizePolicy::ShrinkFlag;
	auto m = model();
	auto *h = header();
	size_hints_.clear();

	int width = 0;
	for (int i=0, end = m ? m->columnCount() : 0; i < end; ++i) {
		size_hints_.push_back(h->sectionSizeHint(i));
		if (preferred || !auto_hide_cols_.contains(i))
			width += size_hints_.back();
	}

	QSize main_size (width, sizeHintForRow(0) * ((!preferred || !m) ? 2 : m->rowCount()));

	// add size for header
	QSize header_size (0, header()->isVisible() ? header()->height() : 0);

	// add size for scrollbars
	QSize scrollbars (verticalScrollBar()->isVisible() ? verticalScrollBar()->width() : 0,
	                  horizontalScrollBar()->isVisible() ? horizontalScrollBar()->height() : 0);

	return main_size + header_size + scrollbars;
}

void AutoAdjustingTreeView::resizeEvent(QResizeEvent *event)
{
	auto *m = model();
	int columns = m ? m->columnCount() : 0;
	if ((int)size_hints_.size() != columns)
		viewportSizeHint();

	// auto hide/show columns > 0, stretch last column to width
	int available_width = event->size().width();

	int required_width = std::accumulate(size_hints_.begin(), size_hints_.end(), 0);
	std::vector<int> width = size_hints_;

	// if required is larger than available width, try to hide some columns
	QListIterator<int> it(auto_hide_cols_);
	for (it.toBack(); it.hasPrevious() && required_width > available_width;) {
		int section = it.previous();
		required_width -= size_hints_[section];
		width[section] = 0;
	}

	// extend width to current column width if possible
	for (int i=0; i < columns; ++i) {
		if (i == stretch_section_) continue;  // ignore auto-stretch section for now
		int delta = columnWidth(i) > 0 ? columnWidth(i) - width[i] : 0;
		if (delta < 0 || required_width + delta <= available_width) {
			width[i] += delta;
			required_width += delta;
		}
	}

	// stretch section if there is still space available
	if (stretch_section_ >= 0 && stretch_section_ < (int)width.size() &&
	    width[stretch_section_] > 0 && available_width > required_width)
		width[stretch_section_] += available_width - required_width;

	// apply stuff
	for (int i=0; i < columns; ++i)
		setColumnWidth(i, width[i]);
}


TaskListView::TaskListView(QWidget *parent) : AutoAdjustingTreeView(parent)
{
	setStretchSection(0);
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
