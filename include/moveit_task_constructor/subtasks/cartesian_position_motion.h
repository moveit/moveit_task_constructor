// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

namespace moveit::planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface);
}

namespace moveit::task_constructor::subtasks {

class CartesianPositionMotion : public SubTask {
public:
	CartesianPositionMotion(std::string name);

	virtual bool canCompute();

	virtual bool compute();

	void setGroup(std::string group);
	void setLink(std::string link);

	void setMinDistance(double distance);
	void setMaxDistance(double distance);
	void setMinMaxDistance(double min_distance, double max_distance);

	void towards(geometry_msgs::PointStamped goal);
	void along(geometry_msgs::Vector3Stamped direction);

protected:
	std::string group_;

	std::string link_;

	double min_distance_;
	double max_distance_;

	std::string mode_;
	geometry_msgs::PointStamped towards_;
	geometry_msgs::Vector3Stamped along_;

	ros::Publisher pub;
};

}
