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

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/kinematic_constraints/utils.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit {
namespace task_constructor {
namespace solvers {

using PipelineMap = std::unordered_map<std::string, std::string>;

PipelinePlanner::PipelinePlanner(
    const rclcpp::Node::SharedPtr& node, const PipelineMap& pipeline_id_planner_id_map,
    const moveit::planning_pipeline_interfaces::StoppingCriterionFunction& stopping_criterion_callback,
    const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function)
  : node_(node)
  , stopping_criterion_callback_(stopping_criterion_callback)
  , solution_selection_function_(solution_selection_function) {
	// Declare properties of the MotionPlanRequest
	properties().declare<uint>("num_planning_attempts", 1u, "number of planning attempts");
	properties().declare<moveit_msgs::msg::WorkspaceParameters>(
	    "workspace_parameters", moveit_msgs::msg::WorkspaceParameters(), "allowed workspace of mobile base?");

	properties().declare<double>("goal_joint_tolerance", 1e-4, "tolerance for reaching joint goals");
	properties().declare<double>("goal_position_tolerance", 1e-4, "tolerance for reaching position goals");
	properties().declare<double>("goal_orientation_tolerance", 1e-4, "tolerance for reaching orientation goals");
	// Declare properties that configure the planning pipeline
	properties().declare("pipeline_id_planner_id_map", pipeline_id_planner_id_map,
	                     "Set of pipelines and planners used for planning");
}

bool PipelinePlanner::setPlannerId(const std::string& pipeline_name, const std::string& planner_id) {
	// Only set ID if pipeline exists. It is not possible to create new pipelines with this command.
	PipelineMap map = properties().get<PipelineMap>("pipeline_id_planner_id_map");
	auto it = map.find(pipeline_name);
	if (it == map.end()) {
		RCLCPP_ERROR(node_->get_logger(),
		             "PipelinePlanner does not have a pipeline called '%s'. Cannot set pipeline ID '%s'",
		             pipeline_name.c_str(), planner_id.c_str());
		return false;
	}
	it->second = planner_id;
	properties().set("pipeline_id_planner_id_map", std::move(map));
	return true;
}

void PipelinePlanner::setStoppingCriterionFunction(
    const moveit::planning_pipeline_interfaces::StoppingCriterionFunction& stopping_criterion_function) {
	stopping_criterion_callback_ = stopping_criterion_function;
}
void PipelinePlanner::setSolutionSelectionFunction(
    const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function) {
	solution_selection_function_ = solution_selection_function;
}

void PipelinePlanner::init(const core::RobotModelConstPtr& robot_model) {
	// Create planning pipelines once from pipeline_id_planner_id_map.
	// We assume that all parameters required by the pipeline can be found
	// in the namespace of the pipeline name.
	if (planning_pipelines_.empty()) {
		auto map = properties().get<PipelineMap>("pipeline_id_planner_id_map");
		// Create pipeline name vector from the keys of pipeline_id_planner_id_map_
		if (map.empty()) {
			throw std::runtime_error("Cannot initialize PipelinePlanner: pipeline_id_planner_id_map is empty!");
		}

		std::vector<std::string> pipeline_names;
		for (const auto& pipeline_name_planner_id_pair : map) {
			pipeline_names.push_back(pipeline_name_planner_id_pair.first);
		}
		planning_pipelines_ = moveit::planning_pipeline_interfaces::createPlanningPipelineMap(std::move(pipeline_names),
		                                                                                      robot_model, node_);
		// Check if it is still empty
		if (planning_pipelines_.empty()) {
			throw std::runtime_error("Failed to initialize PipelinePlanner: Could not create any valid pipeline");
		}
	} else {
		// Validate that all pipelines use the task's robot model
		for (const auto& pair : planning_pipelines_) {
			if (pair.second->getRobotModel() != robot_model) {
				throw std::runtime_error(
				    "The robot model of the planning pipeline isn't the same as the task's robot model -- ");
			}
		}
	}
}

PlannerInterface::Result PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                               const planning_scene::PlanningSceneConstPtr& to,
                                               const moveit::core::JointModelGroup* joint_model_group, double timeout,
                                               robot_trajectory::RobotTrajectoryPtr& result,
                                               const moveit_msgs::msg::Constraints& path_constraints) {
	// Construct goal constraints from the goal planning scene
	const auto goal_constraints = kinematic_constraints::constructGoalConstraints(
	    to->getCurrentState(), joint_model_group, properties().get<double>("goal_joint_tolerance"));
	return plan(from, joint_model_group, goal_constraints, timeout, result, path_constraints);
}

PlannerInterface::Result PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                               const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                                               const Eigen::Isometry3d& target_eigen,
                                               const moveit::core::JointModelGroup* joint_model_group, double timeout,
                                               robot_trajectory::RobotTrajectoryPtr& result,
                                               const moveit_msgs::msg::Constraints& path_constraints) {
	// Construct a Cartesian target pose from the given target transform and offset
	geometry_msgs::msg::PoseStamped target;
	target.header.frame_id = from->getPlanningFrame();
	target.pose = tf2::toMsg(target_eigen * offset.inverse());

	const auto goal_constraints = kinematic_constraints::constructGoalConstraints(
	    link.getName(), target, properties().get<double>("goal_position_tolerance"),
	    properties().get<double>("goal_orientation_tolerance"));

	return plan(from, joint_model_group, goal_constraints, timeout, result, path_constraints);
}

PlannerInterface::Result PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                               const moveit::core::JointModelGroup* joint_model_group,
                                               const moveit_msgs::msg::Constraints& goal_constraints, double timeout,
                                               robot_trajectory::RobotTrajectoryPtr& result,
                                               const moveit_msgs::msg::Constraints& path_constraints) {
	const auto& map = properties().get<PipelineMap>("pipeline_id_planner_id_map");
	last_successful_planner_ = "Unknown";

	// Create a request for every planning pipeline that should run in parallel
	std::vector<moveit_msgs::msg::MotionPlanRequest> requests;
	requests.reserve(map.size());

	for (const auto& [pipeline_id, planner_id] : map) {
		// Check that requested pipeline exists and skip it if it doesn't exist
		if (planning_pipelines_.count(pipeline_id) == 0) {
			RCLCPP_WARN(node_->get_logger(), "Pipeline '%s' was not created. Skipping.", pipeline_id.c_str());
			continue;
		}
		// Create MotionPlanRequest for pipeline
		moveit_msgs::msg::MotionPlanRequest request;
		request.pipeline_id = pipeline_id;
		request.group_name = joint_model_group->getName();
		request.planner_id = planner_id;
		request.allowed_planning_time = timeout;
		request.start_state.is_diff = true;  // we don't specify an extra start state
		request.num_planning_attempts = properties().get<uint>("num_planning_attempts");
		request.max_velocity_scaling_factor = properties().get<double>("max_velocity_scaling_factor");
		request.max_acceleration_scaling_factor = properties().get<double>("max_acceleration_scaling_factor");
		request.workspace_parameters = properties().get<moveit_msgs::msg::WorkspaceParameters>("workspace_parameters");
		request.goal_constraints.resize(1);
		request.goal_constraints.at(0) = goal_constraints;
		request.path_constraints = path_constraints;
		requests.push_back(request);
	}

	// Run planning pipelines in parallel to create a vector of responses. If a solution selection function is provided,
	// planWithParallelPipelines will return a vector with the single best solution
	std::vector<::planning_interface::MotionPlanResponse> responses =
	    moveit::planning_pipeline_interfaces::planWithParallelPipelines(
	        requests, planning_scene, planning_pipelines_, stopping_criterion_callback_, solution_selection_function_);

	// If solutions exist and the first one is successful
	if (!responses.empty()) {
		auto const solution = responses.at(0);
		if (solution) {
			// Choose the first solution trajectory as response
			result = solution.trajectory;
			last_successful_planner_ = solution.planner_id;
			return { true, "" };
		}
		return { false, solution.error_code.message };
	}
	return { false, "No solutions generated from Pipeline Planner" };
}

}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
