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

#include <stdio.h>

#include "task_panel_p.h"
#include "meta_task_list_model.h"
#include "local_task_model.h"
#include "factory_model.h"
#include "pluginlib_factory.h"
#include "task_display.h"
#include <moveit/task_constructor/stage.h>

#include <rviz/properties/property.h>
#include <rviz/display_group.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/panel_dock_widget.h>
#include <ros/console.h>
#include <QPointer>

namespace moveit_rviz_plugin {

rviz::PanelDockWidget* getStageDockWidget(rviz::WindowManagerInterface* mgr) {
	static QPointer<rviz::PanelDockWidget> widget = nullptr;
	if (!widget && mgr) { // create widget
		StageFactoryPtr factory = getStageFactory();
		if (!factory)
			return nullptr;
		QTreeView *view = new QTreeView();
		view->setModel(new FactoryModel(*factory, factory->mimeType(), view));
		view->expandAll();
		view->setHeaderHidden(true);
		view->setDragDropMode(QAbstractItemView::DragOnly);
		widget = mgr->addPane("Motion Planning Stages", view);
	}
	widget->show();
	return widget;
}


TaskPanel::TaskPanel(QWidget* parent)
  : rviz::Panel(parent), d_ptr(new TaskPanelPrivate(this))
{
	Q_D(TaskPanel);
	// connect signals
	connect(d->actionRemoveTaskTreeRows, SIGNAL(triggered()), this, SLOT(removeSelectedStages()));
	connect(d->actionAddLocalTask, SIGNAL(triggered()), this, SLOT(addTask()));
	connect(d->button_show_stage_dock_widget, SIGNAL(clicked()), this, SLOT(showStageDockWidget()));

	connect(d->tasks_view->selectionModel(), SIGNAL(currentChanged(QModelIndex, QModelIndex)),
	        this, SLOT(onCurrentStageChanged(QModelIndex,QModelIndex)));
}

TaskPanel::~TaskPanel()
{
	delete d_ptr;
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel *q_ptr)
   : q_ptr(q_ptr)
   , settings(new rviz::PropertyTreeModel(new rviz::Property, q_ptr))
{
	setupUi(q_ptr);
	// init tasks view
	tasks_view->setModel(&MetaTaskListModel::instance());

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	solutions_view->setAutoHideSections({1, 2});
	solutions_view->setStretchSection(2);

	// init actions
	tasks_view->addActions({actionRemoveTaskTreeRows, actionAddLocalTask});

	initSettings(settings->getRoot());
	settings_view->setModel(settings);
}

void TaskPanelPrivate::initSettings(rviz::Property* root)
{
}

std::pair<TaskListModel*, TaskDisplay*>
TaskPanelPrivate::getTaskListModel(const QModelIndex &index) const
{
	auto *meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskListModel(index);
}

std::pair<BaseTaskModel*, QModelIndex>
TaskPanelPrivate::getTaskModel(const QModelIndex &index) const
{
	auto *meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskModel(index);
}

void TaskPanel::onInitialize()
{
}

void TaskPanel::save(rviz::Config config) const
{
	rviz::Panel::save(config);
	d_ptr->settings->getRoot()->save(config);
}

void TaskPanel::load(const rviz::Config& config)
{
	rviz::Panel::load(config);
	d_ptr->settings->getRoot()->load(config);
}

void TaskPanel::addTask()
{
	TaskListModel* task_list_model = nullptr;
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	if (!current.isValid()) {
		// create new TaskDisplay
		TaskDisplay *display = new TaskDisplay();
		display->setName("Motion Planning Task");
		vis_manager_->getRootDisplayGroup()->addDisplay(display);
		display->initialize(vis_manager_);
		display->setEnabled(true);

		task_list_model = &display->getTaskListModel();
	} else
		task_list_model = d_ptr->getTaskListModel(current).first;

	task_list_model->insertModel(new LocalTaskModel(task_list_model), current.row());
}

void TaskPanel::showStageDockWidget()
{
	rviz::PanelDockWidget *dock = getStageDockWidget(vis_manager_->getWindowManager());
	if (dock) dock->show();
}

void TaskPanel::removeSelectedStages()
{
	auto *m = d_ptr->tasks_view->model();
	for (const auto &range : d_ptr->tasks_view->selectionModel()->selection())
		m->removeRows(range.top(), range.bottom()-range.top()+1, range.parent());
}

void TaskPanel::onCurrentStageChanged(const QModelIndex &current, const QModelIndex &previous)
{
	BaseTaskModel *task;
	QModelIndex task_index;
	std::tie(task, task_index) = d_ptr->getTaskModel(current);
	d_ptr->actionRemoveTaskTreeRows->setEnabled(task != nullptr);

	auto *view = d_ptr->solutions_view;
	QItemSelectionModel *sm = view->selectionModel();
	QAbstractItemModel *m = task ? task->getSolutionModel(task_index) : nullptr;
	view->sortByColumn(-1);
	view->setModel(m);
	delete sm;  // we don't store the selection model

	if (m)
		connect(view->selectionModel(), SIGNAL(currentChanged(QModelIndex, QModelIndex)),
		        this, SLOT(onCurrentSolutionChanged(QModelIndex,QModelIndex)));
}

void TaskPanel::onCurrentSolutionChanged(const QModelIndex &current, const QModelIndex &previous)
{
	if (!current.isValid())
		return;

	TaskDisplay *display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	Q_ASSERT(display);

	BaseTaskModel *task;
	QModelIndex task_index;
	std::tie(task, task_index) = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex());
	Q_ASSERT(task);

	const DisplaySolutionPtr &solution = task->getSolution(current);
	if (!solution)
		return;

	display->showTrajectory(solution);
}

}

#include "moc_task_panel.cpp"
