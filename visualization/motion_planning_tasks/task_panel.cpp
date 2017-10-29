#include <stdio.h>

#include "task_panel.h"
#include "ui_task_panel.h"
#include "task_model.h"

#include <moveit_task_constructor/Task.h>
#include <moveit_task_constructor/introspection.h>
#include <moveit/background_processing/background_processing.h>
#include "mainloop_processing.h"

#include <ros/ros.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property_tree_widget.h>

#include <QTimer>
namespace moveit_rviz_plugin {

class TaskPanelPrivate : public Ui_TaskPanel {
public:
	TaskPanelPrivate(TaskPanel *q_ptr);

	void initSettings(rviz::Property *root);
	void processTaskMessage(const moveit_task_constructor::TaskConstPtr &msg);

	// private slots
	void _q_changedTaskMonitorTopic();

	TaskPanel* q_ptr;
	ros::NodeHandle nh;
	ros::Subscriber task_monitor_sub;

	// task model
	TaskModel* tasks_model;

	// settings
	rviz::PropertyTreeModel* settings;
	rviz::RosTopicProperty* task_monitor_topic_property_;

	moveit::tools::BackgroundProcessing background_process_;
	moveit::tools::MainLoopProcessing mainloop_jobs_;
	QTimer mainloop_timer_; // timer to trigger mainloop jobs;
};


TaskPanel::TaskPanel(QWidget* parent)
  : rviz::Panel(parent), d_ptr(new TaskPanelPrivate(this))
{
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel *q_ptr)
   : q_ptr(q_ptr)
   , tasks_model(new TaskModel(q_ptr))
   , settings(new rviz::PropertyTreeModel(new rviz::Property))
{
	setupUi(q_ptr);
	initSettings(settings->getRoot());
	settings_view->setModel(settings);
	tasks_view->setModel(tasks_model);

	// frequently run mainloop jobs
	QObject::connect(&mainloop_timer_, &QTimer::timeout,
	                 &mainloop_jobs_, &moveit::tools::MainLoopProcessing::executeJobs);
	mainloop_timer_.start(100);
}

void TaskPanelPrivate::initSettings(rviz::Property* root)
{
	task_monitor_topic_property_ =
	      new rviz::RosTopicProperty("Task Monitor Topic", DEFAULT_TASK_MONITOR_TOPIC,
	                                 ros::message_traits::datatype<moveit_task_constructor::Task>(),
	                                 "The topic on which task updates (moveit_msgs::Task messages) are received",
	                                 root, SLOT(_q_changedTaskMonitorTopic()), q_ptr);
}

void TaskPanelPrivate::_q_changedTaskMonitorTopic()
{
	task_monitor_sub.shutdown();
	if (!task_monitor_topic_property_->getStdString().empty()) {
		task_monitor_sub = nh.subscribe(task_monitor_topic_property_->getStdString(), 2,
		                                &TaskPanelPrivate::processTaskMessage, this);
	}
}
void TaskPanelPrivate::processTaskMessage(const moveit_task_constructor::TaskConstPtr& msg) {
	// tasks_model needs to be modified in main loop
	mainloop_jobs_.addJob([this, msg](){ tasks_model->processTaskMessage(*msg); });
}

void TaskPanel::onInitialize()
{
	d_ptr->_q_changedTaskMonitorTopic();
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

}

#include "moc_task_panel.cpp"
