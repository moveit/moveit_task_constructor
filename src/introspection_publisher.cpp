#include <moveit_task_constructor/introspection_publisher.h>
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

IntrospectionPublisher::IntrospectionPublisher(const std::string &topic)
{
	ros::NodeHandle n;
	pub_ = n.advertise<moveit_task_constructor::Task>(topic, 1);
}

IntrospectionPublisher &IntrospectionPublisher::instance()
{
	static IntrospectionPublisher instance_;
	return instance_;
}

void IntrospectionPublisher::publish(const Task &t)
{
	moveit_task_constructor::Task msg;
	pub_.publish(t.fillMessage(msg));
}

} }
