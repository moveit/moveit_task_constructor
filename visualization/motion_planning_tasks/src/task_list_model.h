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

#pragma once

#include "pluginlib_factory.h"
#include <moveit/task_constructor/stage.h>
#include <utils/flat_merge_proxy_model.h>

#include <moveit/macros/class_forward.h>
#include <rclcpp/node.hpp>
#include <moveit_task_constructor_msgs/msg/task_description.hpp>
#include <moveit_task_constructor_msgs/msg/task_statistics.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <QAbstractItemModel>
#include <QTreeView>
#include <QTableView>
#include <memory>
#include <QPointer>

namespace rviz_common {
namespace properties {
class PropertyTreeModel;
}
class DisplayContext;
}  // namespace rviz_common

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(DisplaySolution);
MOVEIT_CLASS_FORWARD(RemoteTaskModel);
using StageFactory = PluginlibFactory<moveit::task_constructor::Stage>;
using StageFactoryPtr = std::shared_ptr<StageFactory>;

StageFactoryPtr getStageFactory();

/** Base class to represent a single local or remote Task as a Qt model. */
class BaseTaskModel : public QAbstractItemModel
{
	Q_OBJECT
protected:
	unsigned int flags_ = 0;
	planning_scene::PlanningSceneConstPtr scene_;
	rviz_common::DisplayContext* display_context_;

public:
	enum TaskModelFlag
	{
		LOCAL_MODEL = 0x01,
		IS_DESTROYED = 0x02,
		IS_INITIALIZED = 0x04,
		IS_RUNNING = 0x08,
	};

	BaseTaskModel(const planning_scene::PlanningSceneConstPtr& scene, rviz_common::DisplayContext* display_context,
	              QObject* parent = nullptr)
	  : QAbstractItemModel(parent), scene_(scene), display_context_(display_context) {}

	int columnCount(const QModelIndex& /*parent*/ = QModelIndex()) const override { return 4; }
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex& index, int role) const override;

	virtual void setStageFactory(const StageFactoryPtr& /*factory*/) {}
	unsigned int taskFlags() const { return flags_; }
	static QVariant flowIcon(moveit::task_constructor::InterfaceFlags f);

	/// retrieve model index associated with given stage id
	virtual QModelIndex indexFromStageId(size_t id) const = 0;

	/// get solution model for given stage index
	virtual QAbstractItemModel* getSolutionModel(const QModelIndex& index) = 0;
	/// get solution for given solution index
	virtual DisplaySolutionPtr getSolution(const QModelIndex& index) = 0;

	/// get property model for given stage index
	virtual rviz_common::properties::PropertyTreeModel* getPropertyModel(const QModelIndex& index) = 0;
};

/** The TaskListModel maintains a list of multiple BaseTaskModels, local and/or remote.
 *
 *  Each TaskDisplay owns a TaskListModel to maintain the list of tasks published on
 *  a monitoring topic.
 *
 *  Local instances are created by createLocalTaskModel() or in dropMimeData().
 *  Remote instances are discovered via processTaskMessage() / processSolutionMessage().
 */
class TaskListModel : public utils::FlatMergeProxyModel
{
	Q_OBJECT

	// planning scene / robot model used by all tasks in this model
	planning_scene::PlanningSceneConstPtr scene_;
	// rviz::DisplayContext used to show (interactive) markers by the property models
	rviz_common::DisplayContext* display_context_ = nullptr;

	// map from remote task IDs to (active) tasks
	// if task is destroyed remotely, it is marked with flag IS_DESTROYED
	// if task is removed locally from tasks vector, it is marked with a nullptr
	std::map<std::string, RemoteTaskModel*> remote_tasks_;
	// mode reflecting the "Old task handling" setting
	int old_task_handling_;

	// factory used to create stages
	StageFactoryPtr stage_factory_;

	QPointer<BaseTaskModel> active_task_model_;
	QPersistentModelIndex highlighted_row_index_;

	void onRemoveModel(QAbstractItemModel* model) override;

public:
	TaskListModel(QObject* parent = nullptr);
	~TaskListModel() override;

	void setScene(const planning_scene::PlanningSceneConstPtr& scene);
	void setDisplayContext(rviz_common::DisplayContext* display_context);
	void setActiveTaskModel(BaseTaskModel* model) { active_task_model_ = model; }

	int columnCount(const QModelIndex& /*parent*/ = QModelIndex()) const override { return 4; }
	static QVariant horizontalHeader(int column, int role);
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex& index, int role) const override;

	/// process an incoming task description message - only call in Qt's main loop
	void processTaskDescriptionMessage(const moveit_task_constructor_msgs::msg::TaskDescription& msg,
	                                   const std::string& service_name);
	/// process an incoming task description message - only call in Qt's main loop
	void processTaskStatisticsMessage(const moveit_task_constructor_msgs::msg::TaskStatistics& msg);
	/// process an incoming solution message - only call in Qt's main loop
	DisplaySolutionPtr processSolutionMessage(const moveit_task_constructor_msgs::msg::Solution& msg);

	/// insert a TaskModel, pos is relative to modelCount()
	bool insertModel(BaseTaskModel* model, int pos = -1);
	/// create a new LocalTaskModel
	BaseTaskModel* createLocalTaskModel();

	/// providing a StageFactory makes the model accepting drops
	void setStageFactory(const StageFactoryPtr& factory);
	bool dropMimeData(const QMimeData* mime, Qt::DropAction action, int row, int column,
	                  const QModelIndex& parent) override;
	Qt::DropActions supportedDropActions() const override;
	Qt::ItemFlags flags(const QModelIndex& index) const override;

public Q_SLOTS:
	void setOldTaskHandling(int mode);

protected Q_SLOTS:
	void highlightStage(size_t id);
};

class TaskListView : public QTreeView
{
	Q_OBJECT
public:
	TaskListView(QWidget* parent = nullptr);

	void dropEvent(QDropEvent* event) override;

	void setModel(QAbstractItemModel* model) override;
	void dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
	                 const QVector<int>& roles = QVector<int>()) override;

protected:
	void updateColumnWidth();
};

class SolutionListView : public QTreeView
{
	Q_OBJECT
public:
	SolutionListView(QWidget* parent = nullptr);

	void setModel(QAbstractItemModel* model) override;
	void dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
	                 const QVector<int>& roles = QVector<int>()) override;
	void resizeEvent(QResizeEvent* event) override;

protected:
	void updateColumnWidth();
};
}  // namespace moveit_rviz_plugin
