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

#include "pluginlib_factory.h"
#include <moveit/task_constructor/stage.h>
#include <utils/flat_merge_proxy_model.h>

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor_msgs/TaskDescription.h>
#include <moveit_task_constructor_msgs/TaskStatistics.h>
#include <moveit_task_constructor_msgs/Solution.h>

#include <QAbstractItemModel>
#include <QTreeView>
#include <memory>
#include <QPointer>

namespace ros { class ServiceClient; }
namespace rviz {
class PropertyTreeModel;
class DisplayContext;
}

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(DisplaySolution)
MOVEIT_CLASS_FORWARD(RemoteTaskModel)
typedef PluginlibFactory<moveit::task_constructor::Stage> StageFactory;
typedef std::shared_ptr<StageFactory> StageFactoryPtr;

StageFactoryPtr getStageFactory();

/** Base class to represent a single local or remote Task as a Qt model. */
class BaseTaskModel : public QAbstractItemModel {
	Q_OBJECT
protected:
	unsigned int flags_ = 0;

public:
	enum TaskModelFlag {
		LOCAL_MODEL    = 0x01,
		IS_DESTROYED   = 0x02,
		IS_INITIALIZED = 0x04,
		IS_RUNNING     = 0x08,
	};

	BaseTaskModel(QObject *parent = nullptr) : QAbstractItemModel(parent) {}

	int columnCount(const QModelIndex &parent = QModelIndex()) const override { return 3; }
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex &index, int role) const override;

	virtual void setStageFactory(const StageFactoryPtr &factory) {}
	Qt::ItemFlags flags(const QModelIndex &index) const override;
	unsigned int taskFlags() const { return flags_; }
	static QVariant flowIcon(moveit::task_constructor::InterfaceFlags f);

	/// retrieve model index associated with given stage id
	virtual QModelIndex indexFromStageId(size_t id) const = 0;

	/// get solution model for given stage index
	virtual QAbstractItemModel* getSolutionModel(const QModelIndex& index) = 0;
	/// get solution for given solution index
	virtual DisplaySolutionPtr getSolution(const QModelIndex &index) = 0;

	/// get property model for given stage index
	virtual rviz::PropertyTreeModel* getPropertyModel(const QModelIndex& index) = 0;
};


/** The TaskListModel maintains a list of multiple BaseTaskModels, local and/or remote.
 *
 *  Each TaskDisplay owns a TaskListModel to maintain the list of tasks published on
 *  a monitoring topic.
 *
 *  Local instances are created by createLocalTaskModel() or in dropMimeData().
 *  Remote instances are discovered via processTaskMessage() / processSolutionMessage().
 */
class TaskListModel : public utils::FlatMergeProxyModel {
	Q_OBJECT

	// planning scene / robot model used by all tasks in this model
	planning_scene::PlanningSceneConstPtr scene_;
	// rviz::DisplayContext used to show (interactive) markers by the property models
	rviz::DisplayContext* display_context_ = nullptr;

	// map from remote task IDs to tasks
	// if task is destroyed remotely, it is marked with flag IS_DESTROYED
	// if task is removed locally from tasks vector, it is marked with a nullptr
	std::map<std::string, RemoteTaskModel*> remote_tasks_;
	ros::ServiceClient* get_solution_client_ = nullptr;

	// factory used to create stages
	StageFactoryPtr stage_factory_;

	QPointer<BaseTaskModel> active_task_model_;
	QPersistentModelIndex highlighted_row_index_;

	void onRemoveModel(QAbstractItemModel *model) override;

public:
	TaskListModel(QObject *parent = nullptr);
	~TaskListModel();

	void setScene(const planning_scene::PlanningSceneConstPtr& scene);
	void setDisplayContext(rviz::DisplayContext* display_context);
	void setSolutionClient(ros::ServiceClient* client);
	void setActiveTaskModel(BaseTaskModel* model) { active_task_model_ = model; }

	int columnCount(const QModelIndex &parent = QModelIndex()) const override { return 3; }
	static QVariant horizontalHeader(int column, int role);
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex &index, int role) const override;

	/// process an incoming task description message - only call in Qt's main loop
	void processTaskDescriptionMessage(const std::string &id, const moveit_task_constructor_msgs::TaskDescription &msg);
	/// process an incoming task description message - only call in Qt's main loop
	void processTaskStatisticsMessage(const std::string &id, const moveit_task_constructor_msgs::TaskStatistics &msg);
	/// process an incoming solution message - only call in Qt's main loop
	DisplaySolutionPtr processSolutionMessage(const std::string &id, const moveit_task_constructor_msgs::Solution &msg);

	/// insert a TaskModel, pos is relative to modelCount()
	bool insertModel(BaseTaskModel* model, int pos = -1);
	/// create a new LocalTaskModel
	BaseTaskModel* createLocalTaskModel();

	/// providing a StageFactory makes the model accepting drops
	void setStageFactory(const StageFactoryPtr &factory);
	bool dropMimeData(const QMimeData *mime, Qt::DropAction action, int row, int column, const QModelIndex &parent) override;
	Qt::DropActions supportedDropActions() const override;
	Qt::ItemFlags flags(const QModelIndex &index) const override;

protected Q_SLOTS:
	void highlightStage(size_t id);
};


class AutoAdjustingTreeView : public QTreeView {
	Q_OBJECT
	Q_PROPERTY(int stretchSection READ stretchSection WRITE setStretchSection)

	mutable std::vector<int> size_hints_;  // size hints for sections
	QList<int> auto_hide_cols_;  // auto-hiding sections
	int stretch_section_ = -1;

public:
	AutoAdjustingTreeView(QWidget *parent = nullptr);

	int stretchSection() const { return stretch_section_; }
	void setStretchSection(int section);

	void setAutoHideSections(const QList<int> &sections);

	void setModel(QAbstractItemModel *model) override;
	QSize viewportSizeHint() const override;
	void resizeEvent(QResizeEvent *event) override;
};


class TaskListView : public AutoAdjustingTreeView {
	Q_OBJECT
public:
	TaskListView(QWidget *parent = nullptr);

	void dropEvent(QDropEvent *event) override;
};

}
