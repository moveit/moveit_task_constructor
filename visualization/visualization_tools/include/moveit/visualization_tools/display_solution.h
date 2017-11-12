#pragma once

#include <moveit_task_constructor_msgs/Solution.h>
#include <moveit/macros/class_forward.h>

namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotState)
} }
namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}
namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit_rviz_plugin {

/** Class representing a task solution for display */
class DisplaySolution
{
	size_t steps_;
	std::vector<planning_scene::PlanningSceneConstPtr> scene_;
	std::vector<robot_trajectory::RobotTrajectoryPtr> trajectory_;
	std::vector<std::string> name_;

public:
	size_t getWayPointCount() const { return steps_; }
	bool empty() const { return steps_ == 0; }

	typedef std::pair<size_t, size_t> IndexPair;
	IndexPair indexPair(size_t index) const;

	float getWayPointDurationFromPrevious(const IndexPair& idx_pair) const;
	float getWayPointDurationFromPrevious(size_t index) const {
		if (index >= steps_) return 0.0;
		return getWayPointDurationFromPrevious(indexPair(index));
	}
	const moveit::core::RobotStatePtr& getWayPointPtr(const IndexPair& idx_pair) const;
	const moveit::core::RobotStatePtr& getWayPointPtr(size_t index) const {
		return getWayPointPtr(indexPair(index));
	}
	const planning_scene::PlanningSceneConstPtr& scene(const IndexPair& idx_pair) const;
	const planning_scene::PlanningSceneConstPtr& scene(size_t index) const {
		return scene(indexPair(index));
	}
	const std::string& name(const IndexPair& idx_pair) const;
	const std::string& name(size_t index) const {
		return name(indexPair(index));
	}

	void setFromMessage(const planning_scene::PlanningSceneConstPtr &parent,
	                    const moveit_task_constructor_msgs::Solution& msg);
};

}
