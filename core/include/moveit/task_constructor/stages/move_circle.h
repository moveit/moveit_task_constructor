// TODO

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "pilz_industrial_motion_planner/trajectory_generator_circ.h"


namespace moveit {
namespace core {
class RobotState;
}
}  // namespace moveit
namespace moveit::task_constructor::stages {

class MoveCircle : public PropagatingEitherWay
{
public:
	MoveCircle(const std::string& name = "move circle");

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setGroup(const std::string& group) { setProperty("group", group); }
	
	/// Set the name of the link to move along the circular arc (for solving IK).
	void setLink(const std::string& link) { setProperty("link", link); }

	/// Set the goal pose, which is the endpoint of the circular arc.
	void setGoal(const geometry_msgs::msg::PoseStamped& pose) { setProperty("goal", pose); }

	/// Set the arc constraint, either using the center or interim point.
	void setArcConstraint(const std::pair<std::string, geometry_msgs::msg::PoseStamped>& constraint) {
		setProperty("arc_constraint", constraint);
	};

protected:
	// return false if trajectory shouldn't be stored
	bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& trajectory,
	             Interface::Direction dir) override;

	std::shared_ptr<pilz_industrial_motion_planner::TrajectoryGeneratorCIRC> planner_;
};
}  // namespace moveit::task_constructor:stages
