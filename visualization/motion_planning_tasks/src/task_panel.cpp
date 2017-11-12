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
#include "task_list_model_cache.h"
#include "local_task_model.h"
#include "factory_model.h"
#include "pluginlib_factory.h"
#include <moveit/task_constructor/stage.h>

#include <rviz/properties/property.h>
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
	connect(d->actionRemoveTaskTreeRows, &QAction::triggered, this, &TaskPanel::removeTaskTreeRows);
	connect(d->actionAddLocalTask, &QAction::triggered, this, &TaskPanel::addTask);
	connect(d->button_show_stage_dock_widget, &QToolButton::clicked, this, &TaskPanel::showStageDockWidget);
}

TaskPanel::~TaskPanel()
{
	delete d_ptr;
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel *q_ptr)
   : q_ptr(q_ptr)
   , task_list_model(TaskListModelCache::instance().getGlobalModel())
   , settings(new rviz::PropertyTreeModel(new rviz::Property, q_ptr))
{
	setupUi(q_ptr);
	// init tasks view
	auto factory = getStageFactory();
	if (!factory) button_show_stage_dock_widget->setDisabled(true);
	task_list_model->setStageFactory(factory);
	tasks_view->setModel(task_list_model.get());

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	// init actions
	tasks_view->addActions({actionRemoveTaskTreeRows, actionAddLocalTask});

	initSettings(settings->getRoot());
	settings_view->setModel(settings);
}

void TaskPanelPrivate::initSettings(rviz::Property* root)
{
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
	Q_D(TaskPanel);
	QModelIndex current = d->tasks_view->currentIndex();
	d_ptr->task_list_model->insertTask(new LocalTaskModel(d_ptr->task_list_model.get()), current.row());
}

void TaskPanel::showStageDockWidget()
{
	Q_D(TaskPanel);
	rviz::PanelDockWidget *dock = getStageDockWidget(vis_manager_->getWindowManager());
	if (dock) dock->show();
}

void TaskPanel::removeTaskTreeRows()
{
	Q_D(TaskPanel);
	for (const auto &range : d_ptr->tasks_view->selectionModel()->selection())
		d_ptr->task_list_model->removeRows(range.top(), range.bottom()-range.top()+1, range.parent());
}

}

#include "moc_task_panel.cpp"
