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

/* Authors: Robert Haschke
   Desc:    planner interface
*/

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/task_constructor/properties.h>
#include <Eigen/Geometry>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}
namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}
namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(LinkModel)
MOVEIT_CLASS_FORWARD(RobotModel)
MOVEIT_CLASS_FORWARD(JointModelGroup)
}  // namespace core
}  // namespace moveit

namespace moveit {
namespace task_constructor {
namespace solvers {

MOVEIT_CLASS_FORWARD(PlannerInterface)
class PlannerInterface
{
	// these properties take precedence over stage properties
	PropertyMap properties_;

public:
	PlannerInterface();
	virtual ~PlannerInterface() {}

	PropertyMap& properties() { return properties_; }
	const PropertyMap& properties() const { return properties_; }

	void setProperty(const std::string& name, const boost::any& value) { properties_.set(name, value); }

	virtual void init(const moveit::core::RobotModelConstPtr& robot_model) = 0;

	/// plan trajectory between to robot states
	virtual bool plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
	                  const moveit::core::JointModelGroup* jmg, double timeout,
	                  robot_trajectory::RobotTrajectoryPtr& result,
	                  const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints()) = 0;

	/// plan trajectory from current robot state to Cartesian target
	virtual bool plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
	                  const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg, double timeout,
	                  robot_trajectory::RobotTrajectoryPtr& result,
	                  const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints()) = 0;
};
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
