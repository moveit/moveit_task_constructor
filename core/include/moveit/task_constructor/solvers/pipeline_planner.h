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
   Desc:    plan using MoveIt's PlanningPipeline
*/

#pragma once

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/node.hpp>
#include <moveit/macros/class_forward.h>

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline);
}

namespace moveit {
namespace task_constructor {
namespace solvers {

MOVEIT_CLASS_FORWARD(PipelinePlanner);

/** Use MoveIt's PlanningPipeline to plan a trajectory between to scenes */
class PipelinePlanner : public PlannerInterface
{
public:
	struct Specification
	{
		moveit::core::RobotModelConstPtr model;
		std::string ns{ "ompl" };
		std::string pipeline{ "ompl" };
		std::string adapter_param{ "request_adapters" };
	};

	static planning_pipeline::PlanningPipelinePtr create(const rclcpp::Node::SharedPtr& node,
	                                                     const moveit::core::RobotModelConstPtr& model) {
		Specification spec;
		spec.model = model;
		return create(node, spec);
	}

	static planning_pipeline::PlanningPipelinePtr create(const rclcpp::Node::SharedPtr& node, const Specification& spec);

	/**
	 *
	 * @param node used to load the parameters for the planning pipeline
	 */
	PipelinePlanner(const rclcpp::Node::SharedPtr& node, const std::string& pipeline = "ompl");

	PipelinePlanner(const planning_pipeline::PlanningPipelinePtr& planning_pipeline);

	void setPlannerId(const std::string& planner) { setProperty("planner", planner); }
	std::string getPlannerId() const override { return properties().get<std::string>("planner"); }

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	Result plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
	            const core::JointModelGroup* jmg, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
	            const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

	Result plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
	            const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target,
	            const moveit::core::JointModelGroup* jmg, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
	            const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

protected:
	Result plan(const planning_scene::PlanningSceneConstPtr& from, const moveit_msgs::msg::MotionPlanRequest& req,
	            robot_trajectory::RobotTrajectoryPtr& result);

	std::string pipeline_name_;
	planning_pipeline::PlanningPipelinePtr planner_;
	rclcpp::Node::SharedPtr node_;
};
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
