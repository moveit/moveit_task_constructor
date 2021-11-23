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

/* Author: Robert Haschke */

#pragma once

#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit/macros/class_forward.h>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}
}  // namespace moveit
namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}
namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory);
}
namespace Ogre {
class SceneNode;
}
namespace rviz_common {
class DisplayContext;
}

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(DisplaySolution);
MOVEIT_CLASS_FORWARD(MarkerVisualization);

/** Class representing a task solution for display */
class DisplaySolution
{
	/// number of overall steps
	size_t steps_;
	/// start scene
	planning_scene::PlanningSceneConstPtr start_scene_;

	struct Data
	{
		/// end scene for each sub trajectory
		planning_scene::PlanningSceneConstPtr scene_;
		/// sub trajectories, might be empty
		robot_trajectory::RobotTrajectoryPtr trajectory_;
		/// joints involved in the trajectory
		std::vector<std::string> joints_;
		/// comment of the trajectory
		std::string comment_;
		/// id of creating stage
		uint32_t creator_id_;
		/// rviz markers
		MarkerVisualizationPtr markers_;
	};
	std::vector<Data> data_;

public:
	DisplaySolution() = default;
	/// create DisplaySolution for given sub trajectory of master
	DisplaySolution(const DisplaySolution& master, uint32_t sub);

	size_t numSubSolutions() const { return data_.size(); }

	size_t getWayPointCount() const { return steps_; }
	bool empty() const { return steps_ == 0; }

	/// pair of trajectory part and way point index within part
	using IndexPair = std::pair<size_t, size_t>;
	IndexPair indexPair(size_t index) const;

	float getWayPointDurationFromPrevious(const IndexPair& idx_pair) const;
	float getWayPointDurationFromPrevious(size_t index) const {
		if (index >= steps_)
			return 0.1;  // display time of last waypoint before switching to final scene
		return getWayPointDurationFromPrevious(indexPair(index));
	}
	const moveit::core::RobotStatePtr& getWayPointPtr(const IndexPair& idx_pair) const;
	const moveit::core::RobotStatePtr& getWayPointPtr(size_t index) const { return getWayPointPtr(indexPair(index)); }
	const planning_scene::PlanningSceneConstPtr& startScene() const { return start_scene_; }
	const planning_scene::PlanningSceneConstPtr& scene(const IndexPair& idx_pair) const;
	const planning_scene::PlanningSceneConstPtr& scene(size_t index) const {
		if (index >= steps_)
			return data_.back().scene_;
		return scene(indexPair(index));
	}
	const std::string& comment(const IndexPair& idx_pair) const;
	const std::string& comment(size_t index) const { return comment(indexPair(index)); }
	uint32_t creatorId(const IndexPair& idx_pair) const;

	const MarkerVisualizationPtr markers(const IndexPair& idx_pair) const;
	const MarkerVisualizationPtr markers(size_t index) const { return markers(indexPair(index)); }
	const MarkerVisualizationPtr markersOfSubTrajectory(size_t index) const { return data_.at(index).markers_; }

	void setFromMessage(const planning_scene::PlanningScenePtr& start_scene,
	                    const moveit_task_constructor_msgs::msg::Solution& msg);
	void fillMessage(moveit_task_constructor_msgs::msg::Solution& msg) const;
};
}  // namespace moveit_rviz_plugin
