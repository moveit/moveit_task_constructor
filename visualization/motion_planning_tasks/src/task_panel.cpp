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
#include <moveit/visualization_tools/task_solution_visualization.h>
#include <moveit/task_constructor/stage.h>

#include <rviz/properties/property.h>
#include <rviz/display_group.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_frame.h>
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


// TaskPanel singleton
static QPointer<TaskPanel> singleton_;
// count active TaskDisplays
static uint display_count_ = 0;


TaskPanel::TaskPanel(QWidget* parent)
  : rviz::Panel(parent), d_ptr(new TaskPanelPrivate(this))
{
	Q_D(TaskPanel);

	d->tasks_widget = new TaskView(this);
	d->settings_widget = new TaskSettings(this);
	layout()->addWidget(d->tasks_widget);
	layout()->addWidget(d->settings_widget);
	connect(d->tasks_widget, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

	connect(d->button_show_stage_dock_widget, SIGNAL(clicked()), this, SLOT(showStageDockWidget()));
	connect(d->button_show_settings, SIGNAL(toggled(bool)), d->settings_widget, SLOT(setVisible(bool)));
	d->settings_widget->setVisible(d->button_show_settings->isChecked());

	// if still undefined, this becomes the global instance
	if (singleton_.isNull())
		singleton_ = this;
}

TaskPanel::~TaskPanel()
{
	delete d_ptr;
}

void TaskPanel::incDisplayCount(rviz::WindowManagerInterface* window_manager)
{
	++display_count_;

	rviz::VisualizationFrame* vis_frame = dynamic_cast<rviz::VisualizationFrame*>(window_manager);
	if (singleton_ || !vis_frame)
		return; // already define, nothing to do

	window_manager->addPane("Motion Planning Tasks", new TaskPanel());
	singleton_->initialize(vis_frame->getManager());
}

void TaskPanel::decDisplayCount()
{
	Q_ASSERT(display_count_ > 0);
	if (--display_count_ == 0 && singleton_)
		singleton_->deleteLater();
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel *q_ptr)
   : q_ptr(q_ptr)
{
	setupUi(q_ptr);
	button_show_stage_dock_widget->setEnabled(bool(getStageFactory()));
}

void TaskPanel::onInitialize()
{
	d_ptr->window_manager_ = vis_manager_->getWindowManager();
}

void TaskPanel::save(rviz::Config config) const
{
	rviz::Panel::save(config);
	d_ptr->tasks_widget->save(config.mapMakeChild("tasks_view"));
}

void TaskPanel::load(const rviz::Config& config)
{
	rviz::Panel::load(config);
	d_ptr->tasks_widget->load(config.mapGetChild("tasks_view"));
}

void TaskPanel::showStageDockWidget()
{
	rviz::PanelDockWidget *dock = getStageDockWidget(d_ptr->window_manager_);
	if (dock) dock->show();
}


// expand all children up to given depth
void setExpanded(QTreeView *view, const QModelIndex &index, bool expand, int depth = -1)
{
	if (!index.isValid())
		return;

	// recursively expand all children
	if (depth != 0) {
		for (int row = 0, rows = index.model()->rowCount(index); row < rows; ++row)
			setExpanded(view, index.child(row, 0), expand, depth-1);
	}

	view->setExpanded(index, expand);
}

TaskViewPrivate::TaskViewPrivate(TaskView *q_ptr)
   : q_ptr(q_ptr)
{
	setupUi(q_ptr);

	MetaTaskListModel *meta_model = &MetaTaskListModel::instance();
	StageFactoryPtr factory = getStageFactory();
	if (factory) meta_model->setMimeTypes( { factory->mimeType() } );
	tasks_view->setModel(meta_model);
	// auto-expand newly-inserted top-level items
	QObject::connect(meta_model, &QAbstractItemModel::rowsInserted, [this](const QModelIndex &parent, int first, int last){
		if (parent.isValid() && !parent.parent().isValid()) {
			for (int row = first; row <= last; ++row) {
				QModelIndex child = parent.child(row, 0);
				// expand inserted items
				setExpanded(tasks_view, child, true);
				// collapse up to first level
				setExpanded(tasks_view, child, false, 1);
				// expand inserted item
				setExpanded(tasks_view, child, true, 0);
			}
			tasks_view->setExpanded(parent, true);  // expand parent group item
		}
	});

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	solutions_view->setAutoHideSections({1, 2});
	solutions_view->setStretchSection(2);

	// init actions
	tasks_view->addActions({actionAddLocalTask, actionRemoveTaskTreeRows});
}

std::pair<TaskListModel*, TaskDisplay*>
TaskViewPrivate::getTaskListModel(const QModelIndex &index) const
{
	auto *meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskListModel(index);
}

std::pair<BaseTaskModel*, QModelIndex>
TaskViewPrivate::getTaskModel(const QModelIndex &index) const
{
	auto *meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskModel(index);
}

void TaskViewPrivate::lock(TaskDisplay* display)
{
	if (locked_display_ && locked_display_ != display) {
		locked_display_->clearMarkers();
		locked_display_->visualization()->unlock();
	}
	locked_display_ = display;
}

TaskView::TaskView(QWidget *parent)
   : QWidget(parent), d_ptr(new TaskViewPrivate(this))
{
	Q_D(TaskView);

	// connect signals
	connect(d->actionRemoveTaskTreeRows, SIGNAL(triggered()), this, SLOT(removeSelectedStages()));
	connect(d->actionAddLocalTask, SIGNAL(triggered()), this, SLOT(addTask()));

	connect(d->tasks_view->selectionModel(), SIGNAL(currentChanged(QModelIndex, QModelIndex)),
	        this, SLOT(onCurrentStageChanged(QModelIndex,QModelIndex)));

	onCurrentStageChanged(d->tasks_view->currentIndex(), QModelIndex());

	// propagate infos about config changes
	connect(d_ptr->tasks_property_splitter, SIGNAL(splitterMoved(int,int)),this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_solutions_splitter, SIGNAL(splitterMoved(int,int)), this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_view->header(), SIGNAL(sectionResized(int,int,int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sectionResized(int,int,int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sortIndicatorChanged(int,Qt::SortOrder)), this, SIGNAL(configChanged()));
}

TaskView::~TaskView()
{
	delete d_ptr;
}

void TaskView::save(rviz::Config config)
{
	auto writeSplitterSizes = [&config](QSplitter* splitter, const QString& key) {
		rviz::Config group = config.mapMakeChild(key);
		for (int s : splitter->sizes()) {
			rviz::Config item = group.listAppendNew();
			item.setValue(s);
		}
	};
	writeSplitterSizes(d_ptr->tasks_property_splitter, "property_splitter");
	writeSplitterSizes(d_ptr->tasks_solutions_splitter, "solutions_splitter");

	auto writeColumnSizes = [&config](QTreeView* view, const QString& key) {
		rviz::Config group = config.mapMakeChild(key);
		for (int c = 0, end = view->header()->count(); c != end; ++c) {
			rviz::Config item = group.listAppendNew();
			item.setValue(view->columnWidth(c));
		}
	};
	writeColumnSizes(d_ptr->tasks_view, "tasks_view_columns");
	writeColumnSizes(d_ptr->solutions_view, "solutions_view_columns");

	const QHeaderView *view = d_ptr->solutions_view->header();
	rviz::Config group = config.mapMakeChild("solution_sorting");
	group.mapSetValue("column", view->sortIndicatorSection());
	group.mapSetValue("order", view->sortIndicatorOrder());
}

void TaskView::load(const rviz::Config &config)
{
	if (!config.isValid()) return;

	auto readSizes = [&config](const QString& key) {
		rviz::Config group = config.mapGetChild(key);
		QList<int> sizes, empty;
		for (int i = 0; i < group.listLength(); ++i) {
			rviz::Config item = group.listChildAt(i);
			if (item.getType() != rviz::Config::Value) return empty;
			QVariant value = item.getValue();
			bool ok = false;
			int int_value = value.toInt(&ok);
			if (!ok) return empty;
			sizes << int_value;
		}
		return sizes;
	};
	d_ptr->tasks_property_splitter->setSizes(readSizes("property_splitter"));
	d_ptr->tasks_solutions_splitter->setSizes(readSizes("solutions_splitter"));

	int column = 0;
	for (int w : readSizes("tasks_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);
	column = 0;
	for (int w : readSizes("solutions_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);

	QTreeView *view = d_ptr->solutions_view;
	rviz::Config group = config.mapGetChild("solution_sorting");
	int order;
	if (group.mapGetInt("column", &column) && group.mapGetInt("order", &order))
		view->sortByColumn(column, static_cast<Qt::SortOrder>(order));
}

void TaskView::addTask()
{
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	if (!current.isValid()) return;
	bool is_top_level = !current.parent().isValid();

	TaskListModel* task_list_model = d_ptr->getTaskListModel(current).first;
	task_list_model->insertModel(task_list_model->createLocalTaskModel(), is_top_level ? -1 : current.row());

	// select and edit newly inserted model
	if (is_top_level) current = current.model()->index(task_list_model->rowCount()-1, 0, current);
	d_ptr->tasks_view->scrollTo(current);
	d_ptr->tasks_view->setCurrentIndex(current);
	d_ptr->tasks_view->edit(current);
}

void TaskView::removeSelectedStages()
{
	auto *m = d_ptr->tasks_view->model();
	for (const auto &range : d_ptr->tasks_view->selectionModel()->selection())
		m->removeRows(range.top(), range.bottom()-range.top()+1, range.parent());
}

void TaskView::onCurrentStageChanged(const QModelIndex &current, const QModelIndex &previous)
{
	// adding task is allowed on top-level items and sub-top-level items
	d_ptr->actionAddLocalTask->setEnabled(current.isValid() &&
	                                      (!current.parent().isValid() || !current.parent().parent().isValid()));
	// removing stuff is allowed any valid selection except top-level items
	d_ptr->actionRemoveTaskTreeRows->setEnabled(current.isValid() && current.parent().isValid());

	BaseTaskModel *task;
	QModelIndex task_index;
	std::tie(task, task_index) = d_ptr->getTaskModel(current);

	d_ptr->lock(nullptr); // unlocks any locked_display_

	// update the SolutionModel
	QTreeView *view = d_ptr->solutions_view;
	int sort_column = view->header()->sortIndicatorSection();
	Qt::SortOrder sort_order = view->header()->sortIndicatorOrder();

	QItemSelectionModel *sm = view->selectionModel();
	QAbstractItemModel *m = task ? task->getSolutionModel(task_index) : nullptr;
	view->setModel(m);
	view->sortByColumn(sort_column, sort_order);
	if (sm) delete sm;  // we don't store the selection model
	sm = view->selectionModel();

	if (sm) {
		connect(sm, SIGNAL(currentChanged(QModelIndex, QModelIndex)),
		        this, SLOT(onCurrentSolutionChanged(QModelIndex,QModelIndex)));
		connect(sm, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
		        this, SLOT(onSolutionSelectionChanged(QItemSelection, QItemSelection)));
	}

	// update the PropertyModel
	view = d_ptr->property_view;
	sm = view->selectionModel();
	m = task ? task->getPropertyModel(task_index) : nullptr;
	view->setModel(m);
	delete sm;  // we don't store the selection model
}

void TaskView::onCurrentSolutionChanged(const QModelIndex &current, const QModelIndex &previous)
{
	TaskDisplay *display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	d_ptr->lock(display);

	if (!display || !current.isValid())
		return;

	BaseTaskModel *task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	TaskSolutionVisualization* vis = display->visualization();
	const DisplaySolutionPtr& solution = task->getSolution(current);
	display->setSolutionStatus(bool(solution));
	vis->interruptCurrentDisplay();
	vis->showTrajectory(solution, true);
}

void TaskView::onSolutionSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
	QItemSelectionModel *sm = d_ptr->solutions_view->selectionModel();
	const QModelIndexList& selected_rows = sm->selectedRows();

	TaskDisplay *display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	Q_ASSERT(display);
	BaseTaskModel *task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	display->clearMarkers();
	for (const auto& index : selected_rows) {
		const DisplaySolutionPtr &solution = task->getSolution(index);
		display->setSolutionStatus(bool(solution));
		display->addMarkers(solution);
	}
}


TaskSettingsPrivate::TaskSettingsPrivate(TaskSettings *q_ptr)
   : q_ptr(q_ptr)
{
	setupUi(q_ptr);
}

TaskSettings::TaskSettings(QWidget *parent)
   : QWidget(parent), d_ptr(new TaskSettingsPrivate(this))
{
	Q_D(TaskSettings);
}

TaskSettings::~TaskSettings()
{
	delete d_ptr;
}

}

#include "moc_task_panel.cpp"
