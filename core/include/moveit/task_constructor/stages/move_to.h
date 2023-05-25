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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Move to joint-state or Cartesian goal pose
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace moveit {
namespace core {
class RobotState;
}
}  // namespace moveit
namespace moveit {
namespace task_constructor {
namespace stages {

class MoveTo : public PropagatingEitherWay
{
public:
	MoveTo(const std::string& name = "move to",
	       const solvers::PlannerInterfacePtr& planner = solvers::PlannerInterfacePtr());

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setGroup(const std::string& group) { setProperty("group", group); }
	/// setters for IK frame
	void setIKFrame(const geometry_msgs::msg::PoseStamped& pose) { setProperty("ik_frame", pose); }
	void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);
	template <typename T>
	void setIKFrame(const T& pose, const std::string& link) {
		setIKFrame(Eigen::Isometry3d(pose), link);
	}
	void setIKFrame(const std::string& link) { setIKFrame(Eigen::Isometry3d::Identity(), link); }

	/// move link to given pose
	void setGoal(const geometry_msgs::msg::PoseStamped& pose) { setProperty("goal", pose); }

	/// move link to given point, keeping current orientation
	void setGoal(const geometry_msgs::msg::PointStamped& point) { setProperty("goal", point); }

	/// move joint model group to given named pose
	void setGoal(const std::string& named_joint_pose) { setProperty("goal", named_joint_pose); }

	/// move joints specified in msg to their target values
	void setGoal(const moveit_msgs::msg::RobotState& robot_state) { setProperty("goal", robot_state); }

	/// move joints by name to their mapped target value
	void setGoal(const std::map<std::string, double>& joints);

	void setPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
		setProperty("path_constraints", std::move(path_constraints));
	}

protected:
	// return false if trajectory shouldn't be stored
	bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& trajectory,
	             Interface::Direction dir) override;
	bool getJointStateGoal(const boost::any& goal, const core::JointModelGroup* jmg, moveit::core::RobotState& state);
	bool getPoseGoal(const boost::any& goal, const planning_scene::PlanningScenePtr& scene,
	                 Eigen::Isometry3d& target_eigen);
	bool getPointGoal(const boost::any& goal, const Eigen::Isometry3d& ik_pose,
	                  const planning_scene::PlanningScenePtr& scene, Eigen::Isometry3d& target_eigen);

protected:
	solvers::PlannerInterfacePtr planner_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
