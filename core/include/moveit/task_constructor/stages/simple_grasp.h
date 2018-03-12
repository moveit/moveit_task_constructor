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
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace moveit { namespace core { MOVEIT_CLASS_FORWARD(RobotModel) } }
namespace moveit { namespace task_constructor { namespace stages {

class GenerateGraspPose;

/** base class for simple grasping / ungrasping
 *
 * Given a named pre-grasp and grasp posture the stage generates fully-qualified
 * pre-grasp and grasp robot states, connected by a grasping trajectory of the end-effector.
 *
 * Grasping and UnGrasping only differs in the order of subtasks. Hence, the base class
 * provides the common functionality for grasping (forward = true) and ungrasping (forward = false).
 */
class SimpleGraspBase : public SerialContainer {
	moveit::core::RobotModelConstPtr model_;
	GenerateGraspPose* grasp_generator_ = nullptr;

public:
	SimpleGraspBase(const std::string& name, bool forward);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	void setMonitoredStage(Stage* monitored);

	void setEndEffector(const std::string& eef) {
		properties().set<std::string>("eef", eef);
	}
	void setObject(const std::string& object) {
		properties().set<std::string>("object", object);
	}

	void setPreGraspPose(const std::string& pregrasp) {
		properties().set<std::string>("pregrasp", pregrasp);
	}
	void setGraspPose(const std::string& grasp) {
		properties().set<std::string>("grasp", grasp);
	}

	void setIKFrame(const geometry_msgs::PoseStamped &transform) {
		properties().set("ik_frame", transform);
	}
	void setIKFrame(const Eigen::Affine3d& pose, const std::string& link);
	template <typename T>
	void setIKFrame(const T& t, const std::string& link) {
		Eigen::Affine3d transform; transform = t;
		setIKFrame(transform, link);
	}
	void setIKFrame(const std::string& link) {
		setIKFrame(Eigen::Affine3d::Identity(), link);
	}

	void setAngleDelta(double angle_delta) {
		properties().set("angle_delta", angle_delta);
	}

	void setMaxIKSolutions(uint32_t max_ik_solutions) {
		properties().set("max_ik_solutions", max_ik_solutions);
	}
	void setTimeout(double timeout) {
		properties().set("timeout", timeout);
	}
};


/// specialization of SimpleGraspBase to realize grasping
class SimpleGrasp : public SimpleGraspBase {
public:
	SimpleGrasp(const std::string& name = "grasp") : SimpleGraspBase(name, true) {}
};


/// specialization of SimpleGraspBase to realize ungrasping
class SimpleUnGrasp : public SimpleGraspBase {
public:
	SimpleUnGrasp(const std::string& name = "ungrasp") : SimpleGraspBase(name, false) {}
};

} } }
