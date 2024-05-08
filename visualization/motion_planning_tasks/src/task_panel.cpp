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

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <QPointer>
#include <QButtonGroup>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.task_view");

namespace moveit_rviz_plugin {

rviz_common::PanelDockWidget* getStageDockWidget(rviz_common::WindowManagerInterface* mgr) {
	static QPointer<rviz_common::PanelDockWidget> widget = nullptr;
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
static QPointer<TaskPanel> SINGLETON;
// count active TaskDisplays
static uint DISPLAY_COUNT = 0;

TaskPanel::TaskPanel(QWidget* parent) : rviz_common::Panel(parent), d_ptr(new TaskPanelPrivate(this)) {
	Q_D(TaskPanel);

	// sync checked tool button with displayed widget
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
	connect(d->tool_buttons_group, &QButtonGroup::idClicked, d->stackedWidget,
	        [d](int index) { d->stackedWidget->setCurrentIndex(index); });
#else
	connect(d->tool_buttons_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
	        d->stackedWidget, [d](int index) { d->stackedWidget->setCurrentIndex(index); });
#endif
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
	if (SINGLETON.isNull())
		SINGLETON = this;
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

/* Realizing a singleton Panel is a nightmare with rviz...
 * Formally, Panels (as a plugin class) cannot be singleton, because new instances are created on demand.
 * Hence, we decided to use a true singleton for the underlying model only and a fake singleton for the panel.
 * Thus, all panels (in case multiple were created) show the same content.
 * The fake singleton shall ensure that only a single panel is created, even if several displays are created.
 * To this end, the displays request() the need for a panel during their initialization and they release()
 * this need during their destruction. This, in principle, allows to create a panel together with the first
 * display and destroy it when the last display is gone.
 * Obviously, the user can still decide to explicitly delete the panel (or create new ones).

 * The nightmare arises from the order of loading of displays and panels: Displays are loaded first.
 * However, directly creating a panel with the first loaded display doesn't work, because panel loading
 * will create another panel instance later (because there is no singleton support).
 * Hence, we need to postpone the actual panel creation from displays until panel loading is finished as well.
 * This was initially done, by postponing panel creation to TaskDisplay::update(). However, update()
 * will never be called if the display is disabled...
 */

void TaskPanel::request(rviz_common::WindowManagerInterface* window_manager) {
	++DISPLAY_COUNT;

	rviz_common::VisualizationFrame* vis_frame = dynamic_cast<rviz_common::VisualizationFrame*>(window_manager);
	if (SINGLETON || !vis_frame)
		return;  // already defined, nothing to do

	QDockWidget* dock = vis_frame->addPanelByName(
	    "Motion Planning Tasks", "moveit_task_constructor/Motion Planning Tasks", Qt::LeftDockWidgetArea);
	Q_UNUSED(dock);
	assert(dock->widget() == SINGLETON);
}

void TaskPanel::release() {
	Q_ASSERT(DISPLAY_COUNT > 0);
	if (--DISPLAY_COUNT == 0 && SINGLETON)
		SINGLETON->deleteLater();
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel* panel) : q_ptr(panel) {
	setupUi(panel);
	tool_buttons_group = new QButtonGroup(panel);
	tool_buttons_group->setExclusive(true);
	button_show_stage_dock_widget->setEnabled(bool(getStageFactory()));
	button_show_stage_dock_widget->setVisible(false);  // hide for now
	property_root = new rviz_common::properties::Property("Global Settings");
}

void TaskPanel::onInitialize() {
	d_ptr->window_manager_ = getDisplayContext()->getWindowManager();
}

void TaskPanel::save(rviz_common::Config config) const {
	rviz_common::Panel::save(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->save(config.mapMakeChild(w->windowTitle()));
	}
}

void TaskPanel::load(const rviz_common::Config& config) {
	rviz_common::Panel::load(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->load(config.mapGetChild(w->windowTitle()));
	}
}

void TaskPanel::showStageDockWidget() {
	rviz_common::PanelDockWidget* dock = getStageDockWidget(d_ptr->window_manager_);
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
			setExpanded(view, index.model()->index(row, 0, index), expand, depth - 1);
	}

	view->setExpanded(index, expand);
}

TaskViewPrivate::TaskViewPrivate(TaskView* view) : q_ptr(view) {
	setupUi(view);

	node_ = rclcpp::Node::make_shared("task_view_private", "");
	exec_action_client_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
	    node_, "execute_task_solution");

	MetaTaskListModel* meta_model = &MetaTaskListModel::instance();
	StageFactoryPtr factory = getStageFactory();
	if (factory)
		meta_model->setMimeTypes({ factory->mimeType() });
	tasks_view->setModel(meta_model);
	QObject::connect(meta_model, SIGNAL(rowsInserted(QModelIndex, int, int)), q_ptr,
	                 SLOT(configureInsertedModels(QModelIndex, int, int)));

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	actionShowTimeColumn->setChecked(true);

	// init actions
	// TODO(v4hn): add actionAddLocalTask once there is something meaningful to add
	tasks_view->addActions({ /*actionAddLocalTask,*/ actionRemoveTaskTreeRows, actionShowTimeColumn });
}

std::pair<TaskListModel*, TaskDisplay*> TaskViewPrivate::getTaskListModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskListModel(index);
}

std::pair<BaseTaskModel*, QModelIndex> TaskViewPrivate::getTaskModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskModel(index);
}

void TaskViewPrivate::configureTaskListModel(TaskListModel* model) {
	QObject::connect(q_ptr, &TaskView::oldTaskHandlingChanged, model, &TaskListModel::setOldTaskHandling);
	model->setOldTaskHandling(q_ptr->old_task_handling->getOptionInt());
}

void TaskViewPrivate::configureExistingModels() {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	for (int row = meta_model->rowCount() - 1; row >= 0; --row)
		configureTaskListModel(meta_model->getTaskListModel(meta_model->index(row, 0)).first);
}

void TaskViewPrivate::configureInsertedModels(const QModelIndex& parent, int first, int last) {
	if (parent.isValid() && !parent.parent().isValid()) {  // top-level task items inserted
		int expand = q_ptr->initial_task_expand->getOptionInt();
		for (int row = first; row <= last; ++row) {
			// set expanded state of items
			QModelIndex child = parent.model()->index(row, 0, parent);
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

			configureTaskListModel(getTaskListModel(child).first);
		}
		tasks_view->setExpanded(parent, true);  // expand parent group item
	}
}

void TaskViewPrivate::lock(TaskDisplay* display) {
	if (locked_display_ && locked_display_ != display) {
		locked_display_->clearMarkers();
		locked_display_->visualization()->unlock();
	}
	locked_display_ = display;
}

TaskView::TaskView(moveit_rviz_plugin::TaskPanel* parent, rviz_common::properties::Property* root)
  : SubPanel(parent), d_ptr(new TaskViewPrivate(this)) {
	Q_D(TaskView);

	d_ptr->tasks_property_splitter->setStretchFactor(0, 3);
	d_ptr->tasks_property_splitter->setStretchFactor(1, 1);

	// connect signals
	connect(d->actionRemoveTaskTreeRows, SIGNAL(triggered()), this, SLOT(removeSelectedStages()));
	connect(d->actionAddLocalTask, SIGNAL(triggered()), this, SLOT(addTask()));
	connect(d->actionShowTimeColumn, &QAction::triggered, [this](bool checked) { show_time_column->setValue(checked); });

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
	auto configs = new rviz_common::properties::Property("Task View Settings", QVariant(), QString(), root);
	initial_task_expand = new rviz_common::properties::EnumProperty(
	    "Task Expansion", "All Expanded", "Configure how to initially expand new tasks", configs);
	initial_task_expand->addOption("Top-level Expanded", EXPAND_TOP);
	initial_task_expand->addOption("All Expanded", EXPAND_ALL);
	initial_task_expand->addOption("All Closed", EXPAND_NONE);

	old_task_handling = new rviz_common::properties::EnumProperty(
	    "Old task handling", "Keep", "Configure what to do with old tasks whose solutions cannot be queried anymore",
	    configs);
	old_task_handling->addOption("Keep", OLD_TASK_KEEP);
	old_task_handling->addOption("Replace", OLD_TASK_REPLACE);
	old_task_handling->addOption("Remove", OLD_TASK_REMOVE);
	connect(old_task_handling, &rviz_common::properties::Property::changed, this, &TaskView::onOldTaskHandlingChanged);

	show_time_column =
	    new rviz_common::properties::BoolProperty("Show Computation Times", true, "Show the 'time' column", configs);
	connect(show_time_column, &rviz_common::properties::Property::changed, this, &TaskView::onShowTimeChanged);

	d_ptr->configureExistingModels();
}

TaskView::~TaskView() {
	delete d_ptr;
}

void TaskView::save(rviz_common::Config config) {
	auto write_splitter_sizes = [&config](QSplitter* splitter, const QString& key) {
		rviz_common::Config group = config.mapMakeChild(key);
		for (int s : splitter->sizes()) {
			rviz_common::Config item = group.listAppendNew();
			item.setValue(s);
		}
	};
	write_splitter_sizes(d_ptr->tasks_property_splitter, "property_splitter");
	write_splitter_sizes(d_ptr->tasks_solutions_splitter, "solutions_splitter");

	auto write_column_sizes = [&config](QHeaderView* view, const QString& key) {
		rviz_common::Config group = config.mapMakeChild(key);
		for (int c = 0, end = view->count(); c != end; ++c) {
			rviz_common::Config item = group.listAppendNew();
			item.setValue(view->sectionSize(c));
		}
	};
	write_column_sizes(d_ptr->tasks_view->header(), "tasks_view_columns");
	write_column_sizes(d_ptr->solutions_view->header(), "solutions_view_columns");

	const QHeaderView* view = d_ptr->solutions_view->header();
	rviz_common::Config group = config.mapMakeChild("solution_sorting");
	group.mapSetValue("column", view->sortIndicatorSection());
	group.mapSetValue("order", view->sortIndicatorOrder());
}

void TaskView::load(const rviz_common::Config& config) {
	if (!config.isValid())
		return;

	auto read_sizes = [&config](const QString& key) {
		rviz_common::Config group = config.mapGetChild(key);
		QList<int> sizes, empty;
		for (int i = 0; i < group.listLength(); ++i) {
			rviz_common::Config item = group.listChildAt(i);
			if (item.getType() != rviz_common::Config::Value)
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
	d_ptr->tasks_property_splitter->setSizes(read_sizes("property_splitter"));
	d_ptr->tasks_solutions_splitter->setSizes(read_sizes("solutions_splitter"));

	int column = 0;
	for (int w : read_sizes("tasks_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);
	column = 0;
	for (int w : read_sizes("solutions_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);

	QTreeView* view = d_ptr->solutions_view;
	rviz_common::Config group = config.mapGetChild("solution_sorting");
	int order = 0;
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

void TaskView::onCurrentStageChanged(const QModelIndex& current, const QModelIndex& /*previous*/) {
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
	if (view->model() != m) {
		view->setModel(m);
		view->sortByColumn(sort_column, sort_order);
		delete sm;  // we don't store the selection model

		sm = view->selectionModel();
		connect(sm, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
		        SLOT(onCurrentSolutionChanged(QModelIndex, QModelIndex)));
		connect(sm, SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this,
		        SLOT(onSolutionSelectionChanged(QItemSelection, QItemSelection)));
	}

	// update the PropertyModel
	view = d_ptr->property_view;
	sm = view->selectionModel();
	m = task ? task->getPropertyModel(task_index) : nullptr;
	if (view->model() != m) {
		view->setModel(m);
		delete sm;  // we don't store the selection model
	}
}

void TaskView::onCurrentSolutionChanged(const QModelIndex& current, const QModelIndex& /*previous*/) {
	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	d_ptr->lock(display);

	if (!display || !current.isValid())
		return;

	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	TaskSolutionVisualization* vis = display->visualization();
	DisplaySolutionPtr solution;
	try {
		solution = task->getSolution(current);
		display->setSolutionStatus(bool(solution));
	} catch (const std::invalid_argument& e) {
		RCLCPP_ERROR_STREAM(LOGGER, e.what());
		display->setSolutionStatus(false, e.what());
	}
	vis->interruptCurrentDisplay();
	vis->showTrajectory(solution, true);
}

void TaskView::onSolutionSelectionChanged(const QItemSelection& /*selected*/, const QItemSelection& /*deselected*/) {
	QItemSelectionModel* sm = d_ptr->solutions_view->selectionModel();
	const QModelIndexList& selected_rows = sm->selectedRows();

	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	Q_ASSERT(display);
	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	display->clearMarkers();
	for (const auto& index : selected_rows) {
		DisplaySolutionPtr solution;
		try {
			solution = task->getSolution(index);
			display->setSolutionStatus(bool(solution));
		} catch (const std::invalid_argument& e) {
			RCLCPP_ERROR_STREAM(LOGGER, e.what());
			display->setSolutionStatus(false, e.what());
		}
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

	if (!d_ptr->exec_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_ERROR(LOGGER, "Failed to connect to the 'execute_task_solution' action server");
		return;
	}

	moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal;
	solution->fillMessage(goal.solution);
	auto goal_handle_future = d_ptr->exec_action_client_->async_send_goal(goal);
	if (rclcpp::spin_until_future_complete(d_ptr->node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(LOGGER, "send goal call failed");
		return;
	}
	const auto& goal_handle = goal_handle_future.get();
	if (!goal_handle)
		RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
}

void TaskView::onShowTimeChanged() {
	auto* header = d_ptr->tasks_view->header();
	bool show = show_time_column->getBool();
	if (header->count() > 3)
		d_ptr->tasks_view->header()->setSectionHidden(3, !show);
	d_ptr->actionShowTimeColumn->setChecked(show);
}

void TaskView::onOldTaskHandlingChanged() {
	Q_EMIT oldTaskHandlingChanged(old_task_handling->getOptionInt());
}

GlobalSettingsWidgetPrivate::GlobalSettingsWidgetPrivate(GlobalSettingsWidget* widget,
                                                         rviz_common::properties::Property* root)
  : q_ptr(widget) {
	setupUi(widget);
	properties = new rviz_common::properties::PropertyTreeModel(root, widget);
	view->setModel(properties);
}

GlobalSettingsWidget::GlobalSettingsWidget(moveit_rviz_plugin::TaskPanel* parent,
                                           rviz_common::properties::Property* root)
  : SubPanel(parent), d_ptr(new GlobalSettingsWidgetPrivate(this, root)) {
	Q_D(GlobalSettingsWidget);

	d->view->expandAll();
	connect(d->properties, &rviz_common::properties::PropertyTreeModel::configChanged, this,
	        &GlobalSettingsWidget::configChanged);
}

GlobalSettingsWidget::~GlobalSettingsWidget() {
	delete d_ptr;
}

void GlobalSettingsWidget::save(rviz_common::Config config) {
	d_ptr->properties->getRoot()->save(config);
}

void GlobalSettingsWidget::load(const rviz_common::Config& config) {
	d_ptr->properties->getRoot()->load(config);
}
}  // namespace moveit_rviz_plugin

#include "moc_task_panel.cpp"
