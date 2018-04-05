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
#include <moveit_msgs/Constraints.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace moveit { namespace task_constructor { namespace stages {

/** Perform a Cartesian motion relative to some link */
class MoveRelative : public PropagatingEitherWay {
public:
	MoveRelative(const std::string& name, const solvers::PlannerInterfacePtr& planner);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool computeForward(const InterfaceState& from) override;
	bool computeBackward(const InterfaceState& to) override;

	void setGroup(const std::string& group);
	void setLink(const std::string& link);

	/// set minimum / maximum distance to move
	void setMinDistance(double distance);
	void setMaxDistance(double distance);
	void setMinMaxDistance(double min_distance, double max_distance);

    void setRelativeJointSpaceGoal(const std::map<std::string, double>& goal);

	void setPathConstraints(moveit_msgs::Constraints path_constraints){
		setProperty("path_constraints", std::move(path_constraints));
	}

	/// perform twist motion on specified link
	void along(const geometry_msgs::TwistStamped& twist);
	/// translate link along given direction
	void along(const geometry_msgs::Vector3Stamped& direction);

protected:
	bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr &scene,
	             SubTrajectory &trajectory, Direction dir);

protected:
	solvers::PlannerInterfacePtr planner_;
    bool joint_space_goal_ = false;
};

} } }
