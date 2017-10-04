// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

namespace moveit {
namespace planning_interface { MOVEIT_CLASS_FORWARD(MoveGroupInterface) }
namespace core { MOVEIT_CLASS_FORWARD(RobotState) }
}

namespace moveit { namespace task_constructor { namespace subtasks {

class CartesianPositionMotion : public PropagatingAnyWay {
public:
	CartesianPositionMotion(std::string name);

	virtual bool computeForward(const InterfaceState &from, planning_scene::PlanningScenePtr &to,
	                            robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost) override;
	virtual bool computeBackward(planning_scene::PlanningScenePtr &from, const InterfaceState &to,
	                             robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost) override;

	void setGroup(std::string group);
	void setLink(std::string link);

	void setMinDistance(double distance);
	void setMaxDistance(double distance);
	void setMinMaxDistance(double min_distance, double max_distance);

	void towards(geometry_msgs::PointStamped goal);
	void along(geometry_msgs::Vector3Stamped direction);

	void setCartesianStepSize(double distance);

protected:
	std::string group_;

	std::string link_;

	double min_distance_;
	double max_distance_;

	enum {
		MODE_ALONG= 1,
		MODE_TOWARDS= 2
	} mode_;

	geometry_msgs::PointStamped towards_;
	geometry_msgs::Vector3Stamped along_;

	double step_size_;

	ros::Publisher pub;

	void _publishTrajectory(const robot_trajectory::RobotTrajectory& trajectory, const moveit::core::RobotState& start);
};

} } }
