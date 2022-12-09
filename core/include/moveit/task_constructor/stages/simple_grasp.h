/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bielefeld University
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

/* Authors: Robert Haschke */

#pragma once

#include <moveit/task_constructor/container.h>
#include <moveit/macros/class_forward.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}
}  // namespace moveit
namespace moveit {
namespace task_constructor {
namespace stages {

class GenerateGraspPose;

/** base class for simple grasping / ungrasping
 *
 * Given a pre-grasp and grasp posture the stage generates a trajectory
 * connecting these two states by a grasping trajectory.
 * The class takes a generator stage that provides the grasp pose and
 * optionally the pre-grasp and grasp postures.
 *
 * Grasping and UnGrasping only differs in the order of subtasks. Hence, the base class
 * provides the common functionality for grasping (forward = true) and ungrasping (forward = false).
 */
class SimpleGraspBase : public SerialContainer
{
	moveit::core::RobotModelConstPtr model_;

protected:
	void setup(std::unique_ptr<Stage>&& generator, bool forward);

public:
	SimpleGraspBase(const std::string& name);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }

	/// set properties of IK solver
	void setIKFrame(const geometry_msgs::msg::PoseStamped& transform) { setProperty("ik_frame", transform); }
	void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);
	void setIKFrame(const std::string& link) { setIKFrame(Eigen::Isometry3d::Identity(), link); }
	/// allow setting IK frame from any type T that converts to Eigen::Isometry3d
	template <typename T>
	void setIKFrame(const T& transform, const std::string& link) {
		setIKFrame(Eigen::Isometry3d(transform), link);
	}

	void setMaxIKSolutions(uint32_t max_ik_solutions) { setProperty("max_ik_solutions", max_ik_solutions); }
};

/// specialization of SimpleGraspBase to realize grasping
class SimpleGrasp : public SimpleGraspBase
{
public:
	SimpleGrasp(Stage::pointer&& generator = Stage::pointer(), const std::string& name = "grasp");
};

/// specialization of SimpleGraspBase to realize ungrasping
class SimpleUnGrasp : public SimpleGraspBase
{
public:
	SimpleUnGrasp(Stage::pointer&& generator = Stage::pointer(), const std::string& name = "ungrasp");
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
