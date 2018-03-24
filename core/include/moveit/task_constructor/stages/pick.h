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

#include <moveit/macros/class_forward.h>
#include <moveit/task_constructor/container.h>
#include <geometry_msgs/TwistStamped.h>

namespace moveit { namespace task_constructor {

namespace solvers {
MOVEIT_CLASS_FORWARD(CartesianPath)
MOVEIT_CLASS_FORWARD(PipelinePlanner)
}

namespace stages {

/** Pick wraps a complete pipeline to pick up an object with a given end effector.
 *
 * Picking consist of the following sub stages:
 * - reaching to the object + "pre-grasp" end effector posture
 * - linearly approaching the object along an approach direction/twist
 * - "grasp" end effector posture
 * - attach object
 * - lift along along a given direction/twist
 *
 * The end effector postures corresponding to pre-grasp and grasp as well as
 * the end effector's Cartesian pose needs to be provided by an external grasp stage.
 */
class Pick : public SerialContainer {
	solvers::CartesianPathPtr cartesian_solver_;
	solvers::PipelinePlannerPtr pipeline_solver_;
	Stage* grasp_stage_ = nullptr;
	Stage* approach_stage_ = nullptr;
	Stage* lift_stage_ = nullptr;

public:
	Pick(Stage::pointer &&grasp_stage, const std::string& name = "pick");

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	void setEndEffector(const std::string& eef) {
		properties().set<std::string>("eef", eef);
	}
	void setObject(const std::string& object) {
		properties().set<std::string>("object", object);
	}

	solvers::CartesianPathPtr cartesianSolver() { return cartesian_solver_; }
	solvers::PipelinePlannerPtr pipelineSolver() { return pipeline_solver_; }

	void setApproachMotion(const geometry_msgs::TwistStamped& motion,
	                       double min_distance, double max_distance);
	void setLiftMotion(const geometry_msgs::TwistStamped& motion,
	                   double min_distance, double max_distance);
};

} } }
