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

/* Authors: Robert Haschke, Sebastian Jahr
   Description: Solver that uses a set of MoveIt PlanningPipelines to solve a given planning problem.
*/

#pragma once

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/planning_pipeline_interfaces/solution_selection_functions.hpp>
#include <moveit/planning_pipeline_interfaces/stopping_criterion_functions.hpp>
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
	/** Simple Constructor to use only a single pipeline
	 * \param [in] node ROS 2 node
	 * \param [in] pipeline_name Name of the planning pipeline to be used. This is also the assumed namespace where the
	 * parameters of this pipeline can be found \param [in] planner_id Planner id to be used for planning
	 */
	PipelinePlanner(const rclcpp::Node::SharedPtr& node, const std::string& pipeline_name = "ompl",
	                const std::string& planner_id = "");

	[[deprecated("Deprecated: Use new constructor implementations.")]]  // clang-format off
	PipelinePlanner(const planning_pipeline::PlanningPipelinePtr& /*planning_pipeline*/){};

	/** \brief Constructor
	 * \param [in] node ROS 2 node
	 * \param [in] pipeline_id_planner_id_map A map containing pairs of planning pipeline name and planner plugin name
	 * for the planning requests
	 * \param [in] planning_pipelines Optional: A map with the pipeline names and initialized corresponding planning
	 * pipelines
	 * \param [in] stopping_criterion_callback Callback function to stop parallel planning pipeline according to a user defined criteria
	 * \param [in] solution_selection_function Callback function to choose the best solution when multiple pipelines are used
	 */
	PipelinePlanner(const rclcpp::Node::SharedPtr& node,
	                const std::unordered_map<std::string, std::string>& pipeline_id_planner_id_map,
	                const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& planning_pipelines =
	                    std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>(),
					const moveit::planning_pipeline_interfaces::StoppingCriterionFunction& stopping_criterion_callback = nullptr,
					const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function =
						&moveit::planning_pipeline_interfaces::getShortestSolution);

	[[deprecated("Replaced with setPlannerId(pipeline_name, planner_id)")]]  // clang-format off
	void setPlannerId(const std::string& /*planner*/) { /* Do nothing */
	}

	/** \brief Set the planner id for a specific planning pipeline for the planning requests
	 * \param [in] pipeline_name Name of the planning pipeline for which the planner id is set
	 * \param [in] planner_id Name of the planner ID that should be used by the planning pipeline
	 * \return true if the pipeline exists and the corresponding ID is set successfully
	 */
	bool setPlannerId(const std::string& pipeline_name, const std::string& planner_id);

	/** \brief Set stopping criterion function for parallel planning
	 * \param [in] stopping_criterion_callback New stopping criterion function that will be used
	*/
	void setStoppingCriterionFunction(const moveit::planning_pipeline_interfaces::StoppingCriterionFunction& stopping_criterion_callback);

	/** \brief Set solution selection function for parallel planning
	 * \param [in] solution_selection_function New solution selection that will be used
	*/
	void setSolutionSelectionFunction(const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function);

	/** \brief This function is called when an MTC task that uses this solver is initialized. If no pipelines are
	 * configured when this function is invoked, the planning pipelines named in the 'pipeline_id_planner_id_map' are
	 * initialized with the given robot model.
	 * \param [in] robot_model A robot model that is used to initialize the
	 * planning pipelines of this solver
	 */
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	/**
	 * \brief Plan a trajectory from a planning scene 'from' to scene 'to'
	 * \param [in] from Start planning scene
	 * \param [in] to Goal planning scene (used to create goal constraints)
	 * \param [in] joint_model_group Group of joints for which this trajectory is created
	 * \param [in] timeout Maximum planning timeout for an individual stage that is using the pipeline planner in seconds
	 * \param [in] result Reference to the location where the created trajectory is stored if planning is successful
	 * \param [in] path_constraints Path contraints for the planning problem
	 * \return true If the solver found a successful solution for the given planning problem
	 */
	bool plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
	          const core::JointModelGroup* joint_model_group, double timeout,
	          robot_trajectory::RobotTrajectoryPtr& result,
	          const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

	/** \brief Plan a trajectory from a planning scene 'from' to a target pose with an offset
	 * \param [in] from Start planning scene
	 * \param [in] link Link for which a target pose is given
	 * \param [in] offset Offset to be applied to a given target pose
	 * \param [in] target Target pose
	 * \param [in] joint_model_group Group of joints for which this trajectory is created
	 * \param [in] timeout Maximum planning timeout for an individual stage that is using the pipeline planner in seconds
	 * \param [in] result Reference to the location where the created trajectory is stored if planning is successful
	 * \param [in] path_constraints Path contraints for the planning problem
	 * \return true If the solver found a successful solution for the given planning problem
	 */
	bool plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
	          const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target,
	          const moveit::core::JointModelGroup* joint_model_group, double timeout,
	          robot_trajectory::RobotTrajectoryPtr& result,
	          const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

	/**
	 * \brief Get planner name
	 * \return Name of the last successful planner
	 */
	std::string getPlannerId() const override;

protected:
	/** \brief Function that actually uses the planning pipelines to solve the given planning problem. It is called by
	 * the public plan function after the goal constraints are generated. This function uses a predefined number of
	 * planning pipelines in parallel to solve the planning problem and choose the best (user-defined) solution.
	 * \param [in] planning_scene Scene for which the planning should be solved
	 * \param [in] joint_model_group
	 * Group of joints for which this trajectory is created
	 * \param [in] goal_constraints Set of constraints that need to
	 * be satisfied by the solution
	 * \param [in] timeout Maximum planning timeout for an individual stage that is using the pipeline planner in seconds
	 * \param [in] result Reference to the location where the created
	 * trajectory is stored if planning is successful
	 * \param [in] path_constraints Path contraints for the planning
	 * problem
	 * \return true If the solver found a successful solution for the given planning problem
	 */
	bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
	          const moveit::core::JointModelGroup* joint_model_group,
	          const moveit_msgs::msg::Constraints& goal_constraints, double timeout,
	          robot_trajectory::RobotTrajectoryPtr& result,
	          const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

	rclcpp::Node::SharedPtr node_;

	std::string last_successful_planner_;

	/** \brief Map of pipeline names (ids) and their corresponding planner ids. The planning problem is solved for every
	 * pipeline-planner pair in this map. If no pipelines are passed via constructor argument, the pipeline names are
	 * used to initialize a set of planning pipelines.  */
	std::unordered_map<std::string, std::string> pipeline_id_planner_id_map_;

	/** \brief Map of pipelines names and planning pipelines. This map is used to quickly search for a requested motion
	 * planning pipeline when during plan(..) */
	std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr> planning_pipelines_;

	moveit::planning_pipeline_interfaces::StoppingCriterionFunction stopping_criterion_callback_;
	moveit::planning_pipeline_interfaces::SolutionSelectionFunction solution_selection_function_;

};
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
