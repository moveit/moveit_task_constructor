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
#include <moveit/visualization_tools/display_solution.h>
#include <moveit/task_constructor/stage.h>

#include <rviz/properties/property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/display_group.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_frame.h>
#include <rviz/panel_dock_widget.h>
#include <ros/console.h>
#include <QPointer>
#include <QButtonGroup>

namespace moveit_rviz_plugin {

rviz::PanelDockWidget* getStageDockWidget(rviz::WindowManagerInterface* mgr) {
	static QPointer<rviz::PanelDockWidget> widget = nullptr;
	if (!widget && mgr) {  // create widget
		StageFactoryPtr factory = getStageFactory();
		if (!factory)
			return nullptr;
		QTreeView* view = new QTreeView();
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

TaskPanel::TaskPanel(QWidget* parent) : rviz::Panel(parent), d_ptr(new TaskPanelPrivate(this)) {
	Q_D(TaskPanel);

	// sync checked tool button with displayed widget
	connect(d->tool_buttons_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
	        d->stackedWidget, [d](int index) { d->stackedWidget->setCurrentIndex(index); });
	connect(d->stackedWidget, &QStackedWidget::currentChanged, d->tool_buttons_group,
	        [d](int index) { d->tool_buttons_group->button(index)->setChecked(true); });

	auto* task_view = new TaskView(this, d->property_root);
	connect(d->button_exec_solution, SIGNAL(clicked()), task_view, SLOT(onExecCurrentSolution()));

	// create sub widgets with corresponding tool buttons
	addSubPanel(task_view, "Tasks View", QIcon(":/icons/tasks.png"));
	d->stackedWidget->setCurrentIndex(0);  // Tasks View is show by default

	// settings widget should come last
	addSubPanel(new GlobalSettingsWidget(this, d->property_root), "Global Settings", QIcon(":/icons/settings.png"));

	connect(d->button_show_stage_dock_widget, SIGNAL(clicked()), this, SLOT(showStageDockWidget()));

	// if still undefined, this becomes the global instance
	if (singleton_.isNull())
		singleton_ = this;
}

TaskPanel::~TaskPanel() {
	delete d_ptr;
}

void TaskPanel::addSubPanel(SubPanel* w, const QString& title, const QIcon& icon) {
	Q_D(TaskPanel);

	auto button = new QToolButton(w);
	button->setToolTip(title);
	button->setIcon(icon);
	button->setCheckable(true);

	int index = d->stackedWidget->count();
	d->tool_buttons_layout->insertWidget(index, button);
	d->tool_buttons_group->addButton(button, index);
	d->stackedWidget->addWidget(w);

	w->setWindowTitle(title);
	connect(w, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
}

void TaskPanel::incDisplayCount(rviz::WindowManagerInterface* window_manager) {
	++display_count_;

	rviz::VisualizationFrame* vis_frame = dynamic_cast<rviz::VisualizationFrame*>(window_manager);
	if (singleton_ || !vis_frame)
		return;  // already define, nothing to do

	QDockWidget* dock =
	    vis_frame->addPanelByName("Motion Planning Tasks", "moveit_task_constructor/Motion Planning Tasks",
	                              Qt::LeftDockWidgetArea, true /* floating */);
	assert(dock->widget() == singleton_);
}

void TaskPanel::decDisplayCount() {
	Q_ASSERT(display_count_ > 0);
	if (--display_count_ == 0 && singleton_)
		singleton_->deleteLater();
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel* q_ptr) : q_ptr(q_ptr) {
	setupUi(q_ptr);
	tool_buttons_group = new QButtonGroup(q_ptr);
	tool_buttons_group->setExclusive(true);
	button_show_stage_dock_widget->setEnabled(bool(getStageFactory()));
	button_show_stage_dock_widget->setToolTip(QStringLiteral("Show available stages"));
	property_root = new rviz::Property("Global Settings");
}

void TaskPanel::onInitialize() {
	d_ptr->window_manager_ = vis_manager_->getWindowManager();
}

void TaskPanel::save(rviz::Config config) const {
	rviz::Panel::save(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->save(config.mapMakeChild(w->windowTitle()));
	}
}

void TaskPanel::load(const rviz::Config& config) {
	rviz::Panel::load(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->load(config.mapGetChild(w->windowTitle()));
	}
}

void TaskPanel::showStageDockWidget() {
	rviz::PanelDockWidget* dock = getStageDockWidget(d_ptr->window_manager_);
	if (dock)
		dock->show();
}

// expand all children up to given depth
void setExpanded(QTreeView* view, const QModelIndex& index, bool expand, int depth = -1) {
	if (!index.isValid())
		return;

	// recursively expand all children
	if (depth != 0) {
		for (int row = 0, rows = index.model()->rowCount(index); row < rows; ++row)
			setExpanded(view, index.child(row, 0), expand, depth - 1);
	}

	view->setExpanded(index, expand);
}

TaskViewPrivate::TaskViewPrivate(TaskView* q_ptr) : q_ptr(q_ptr), exec_action_client_("execute_task_solution") {
	setupUi(q_ptr);

	MetaTaskListModel* meta_model = &MetaTaskListModel::instance();
	StageFactoryPtr factory = getStageFactory();
	if (factory)
		meta_model->setMimeTypes({ factory->mimeType() });
	tasks_view->setModel(meta_model);
	// auto-expand newly-inserted top-level items
	QObject::connect(meta_model, &QAbstractItemModel::rowsInserted,
	                 [this](const QModelIndex& parent, int first, int last) {
		                 if (parent.isValid() && !parent.parent().isValid()) {  // top-level task items inserted
			                 int expand = this->q_ptr->initial_task_expand->getOptionInt();
			                 for (int row = first; row <= last; ++row) {
				                 QModelIndex child = parent.child(row, 0);
				                 if (expand != TaskView::EXPAND_NONE) {
					                 // recursively expand all inserted items
					                 setExpanded(tasks_view, child, true);
				                 }
				                 if (expand == TaskView::EXPAND_TOP) {
					                 // collapse up to first level
					                 setExpanded(tasks_view, child, false, 1);
					                 // expand inserted item
					                 setExpanded(tasks_view, child, true, 0);
				                 }
			                 }
			                 tasks_view->setExpanded(parent, true);  // expand parent group item
		                 }
		              });

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	solutions_view->setAutoHideSections({ 1, 2 });
	solutions_view->setStretchSection(2);

	// init actions
	tasks_view->addActions({ actionAddLocalTask, actionRemoveTaskTreeRows });
}

std::pair<TaskListModel*, TaskDisplay*> TaskViewPrivate::getTaskListModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskListModel(index);
}

std::pair<BaseTaskModel*, QModelIndex> TaskViewPrivate::getTaskModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskModel(index);
}

void TaskViewPrivate::lock(TaskDisplay* display) {
	if (locked_display_ && locked_display_ != display) {
		locked_display_->clearMarkers();
		locked_display_->visualization()->unlock();
	}
	locked_display_ = display;
}

TaskView::TaskView(moveit_rviz_plugin::TaskPanel* parent, rviz::Property* root)
  : SubPanel(parent), d_ptr(new TaskViewPrivate(this)) {
	Q_D(TaskView);

	// connect signals
	connect(d->actionRemoveTaskTreeRows, SIGNAL(triggered()), this, SLOT(removeSelectedStages()));
	connect(d->actionAddLocalTask, SIGNAL(triggered()), this, SLOT(addTask()));

	connect(d->tasks_view->selectionModel(), SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
	        SLOT(onCurrentStageChanged(QModelIndex, QModelIndex)));

	onCurrentStageChanged(d->tasks_view->currentIndex(), QModelIndex());

	// propagate infos about config changes
	connect(d_ptr->tasks_property_splitter, SIGNAL(splitterMoved(int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_solutions_splitter, SIGNAL(splitterMoved(int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_view->header(), SIGNAL(sectionResized(int, int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sectionResized(int, int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sortIndicatorChanged(int, Qt::SortOrder)), this,
	        SIGNAL(configChanged()));

	// configuration settings
	auto configs = new rviz::Property("Task View Settings", QVariant(), QString(), root);
	initial_task_expand =
	    new rviz::EnumProperty("Task Expansion", "All Expanded", "Configure how to initially expand new tasks", configs);
	initial_task_expand->addOption("Top-level Expanded", EXPAND_TOP);
	initial_task_expand->addOption("All Expanded", EXPAND_ALL);
	initial_task_expand->addOption("All Closed", EXPAND_NONE);
}

TaskView::~TaskView() {
	delete d_ptr;
}

void TaskView::save(rviz::Config config) {
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

	const QHeaderView* view = d_ptr->solutions_view->header();
	rviz::Config group = config.mapMakeChild("solution_sorting");
	group.mapSetValue("column", view->sortIndicatorSection());
	group.mapSetValue("order", view->sortIndicatorOrder());
}

void TaskView::load(const rviz::Config& config) {
	if (!config.isValid())
		return;

	auto readSizes = [&config](const QString& key) {
		rviz::Config group = config.mapGetChild(key);
		QList<int> sizes, empty;
		for (int i = 0; i < group.listLength(); ++i) {
			rviz::Config item = group.listChildAt(i);
			if (item.getType() != rviz::Config::Value)
				return empty;
			QVariant value = item.getValue();
			bool ok = false;
			int int_value = value.toInt(&ok);
			if (!ok)
				return empty;
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

	QTreeView* view = d_ptr->solutions_view;
	rviz::Config group = config.mapGetChild("solution_sorting");
	int order;
	if (group.mapGetInt("column", &column) && group.mapGetInt("order", &order))
		view->sortByColumn(column, static_cast<Qt::SortOrder>(order));
}

void TaskView::addTask() {
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	if (!current.isValid())
		return;
	bool is_top_level = !current.parent().isValid();

	TaskListModel* task_list_model = d_ptr->getTaskListModel(current).first;
	task_list_model->insertModel(task_list_model->createLocalTaskModel(), is_top_level ? -1 : current.row());

	// select and edit newly inserted model
	if (is_top_level)
		current = current.model()->index(task_list_model->rowCount() - 1, 0, current);
	d_ptr->tasks_view->scrollTo(current);
	d_ptr->tasks_view->setCurrentIndex(current);
	d_ptr->tasks_view->edit(current);
}

void TaskView::removeSelectedStages() {
	auto* m = d_ptr->tasks_view->model();
	for (const auto& range : d_ptr->tasks_view->selectionModel()->selection())
		m->removeRows(range.top(), range.bottom() - range.top() + 1, range.parent());
}

void TaskView::onCurrentStageChanged(const QModelIndex& current, const QModelIndex& previous) {
	// adding task is allowed on top-level items and sub-top-level items
	d_ptr->actionAddLocalTask->setEnabled(current.isValid() &&
	                                      (!current.parent().isValid() || !current.parent().parent().isValid()));
	// removing stuff is allowed any valid selection except top-level items
	d_ptr->actionRemoveTaskTreeRows->setEnabled(current.isValid() && current.parent().isValid());

	BaseTaskModel* task;
	QModelIndex task_index;
	std::tie(task, task_index) = d_ptr->getTaskModel(current);

	d_ptr->lock(nullptr);  // unlocks any locked_display_

	// update the SolutionModel
	QTreeView* view = d_ptr->solutions_view;
	int sort_column = view->header()->sortIndicatorSection();
	Qt::SortOrder sort_order = view->header()->sortIndicatorOrder();

	QItemSelectionModel* sm = view->selectionModel();
	QAbstractItemModel* m = task ? task->getSolutionModel(task_index) : nullptr;
	view->setModel(m);
	view->sortByColumn(sort_column, sort_order);
	delete sm;  // we don't store the selection model
	sm = view->selectionModel();

	if (sm) {
		connect(sm, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
		        SLOT(onCurrentSolutionChanged(QModelIndex, QModelIndex)));
		connect(sm, SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this,
		        SLOT(onSolutionSelectionChanged(QItemSelection, QItemSelection)));
	}

	// update the PropertyModel
	view = d_ptr->property_view;
	sm = view->selectionModel();
	m = task ? task->getPropertyModel(task_index) : nullptr;
	view->setModel(m);
	delete sm;  // we don't store the selection model
}

void TaskView::onCurrentSolutionChanged(const QModelIndex& current, const QModelIndex& previous) {
	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	d_ptr->lock(display);

	if (!display || !current.isValid())
		return;

	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	TaskSolutionVisualization* vis = display->visualization();
	const DisplaySolutionPtr& solution = task->getSolution(current);
	display->setSolutionStatus(bool(solution));
	vis->interruptCurrentDisplay();
	vis->showTrajectory(solution, true);
}

void TaskView::onSolutionSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected) {
	QItemSelectionModel* sm = d_ptr->solutions_view->selectionModel();
	const QModelIndexList& selected_rows = sm->selectedRows();

	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	Q_ASSERT(display);
	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	display->clearMarkers();
	for (const auto& index : selected_rows) {
		const DisplaySolutionPtr& solution = task->getSolution(index);
		display->setSolutionStatus(bool(solution));
		display->addMarkers(solution);
	}
}

void TaskView::onExecCurrentSolution() const {
	const QModelIndex& current = d_ptr->solutions_view->currentIndex();
	if (!current.isValid())
		return;

	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	const DisplaySolutionPtr& solution = task->getSolution(current);

	if (!d_ptr->exec_action_client_.waitForServer(ros::Duration(0.1))) {
		ROS_ERROR("Failed to connect to task execution action");
		return;
	}

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	solution->fillMessage(goal.solution);
	d_ptr->exec_action_client_.sendGoal(goal);
}

GlobalSettingsWidgetPrivate::GlobalSettingsWidgetPrivate(GlobalSettingsWidget* q_ptr, rviz::Property* root)
  : q_ptr(q_ptr) {
	setupUi(q_ptr);
	properties = new rviz::PropertyTreeModel(root, q_ptr);
	view->setModel(properties);
}

GlobalSettingsWidget::GlobalSettingsWidget(moveit_rviz_plugin::TaskPanel* parent, rviz::Property* root)
  : SubPanel(parent), d_ptr(new GlobalSettingsWidgetPrivate(this, root)) {
	Q_D(GlobalSettingsWidget);
	connect(d->properties, &rviz::PropertyTreeModel::configChanged, this, &GlobalSettingsWidget::configChanged);
}

GlobalSettingsWidget::~GlobalSettingsWidget() {
	delete d_ptr;
}

void GlobalSettingsWidget::save(rviz::Config config) {
	d_ptr->properties->getRoot()->save(config);
}

void GlobalSettingsWidget::load(const rviz::Config& config) {
	d_ptr->properties->getRoot()->load(config);
}
}

#include "moc_task_panel.cpp"
