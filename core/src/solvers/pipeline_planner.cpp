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
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace solvers {

PipelinePlanner::PipelinePlanner() {
	auto& p = properties();
	p.declare<std::string>("planner", "", "planner id");

	p.declare<uint>("num_planning_attempts", 1u, "number of planning attempts");
	p.declare<moveit_msgs::WorkspaceParameters>("workspace_parameters", moveit_msgs::WorkspaceParameters(),
	                                            "allowed workspace of mobile base?");

	p.declare<double>("goal_joint_tolerance", 1e-4, "tolerance for reaching joint goals");
	p.declare<double>("goal_position_tolerance", 1e-4, "tolerance for reaching position goals");
	p.declare<double>("goal_orientation_tolerance", 1e-4, "tolerance for reaching orientation goals");

	p.declare<bool>("display_motion_plans", false,
	                "publish generated solutions on topic " + planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC);
	p.declare<bool>("publish_planning_requests", false,
	                "publish motion planning requests on topic " +
	                    planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC);
}

PipelinePlanner::PipelinePlanner(const planning_pipeline::PlanningPipelinePtr& planning_pipeline) : PipelinePlanner() {
	planner_ = planning_pipeline;
}

void PipelinePlanner::init(const core::RobotModelConstPtr& robot_model) {
	if (!planner_) {
		planner_ = Task::createPlanner(robot_model);
	} else if (robot_model != planner_->getRobotModel()) {
		throw std::runtime_error(
		    "The robot model of the planning pipeline isn't the same as the task's robot model -- "
		    "use Task::setRobotModel for setting the robot model when using custom planning pipeline");
	}
	planner_->displayComputedMotionPlans(properties().get<bool>("display_motion_plans"));
	planner_->publishReceivedRequests(properties().get<bool>("publish_planning_requests"));
}

void initMotionPlanRequest(moveit_msgs::MotionPlanRequest& req, const PropertyMap& p,
                           const moveit::core::JointModelGroup* jmg, double timeout) {
	req.group_name = jmg->getName();
	req.planner_id = p.get<std::string>("planner");
	req.allowed_planning_time = timeout;
	req.start_state.is_diff = true;  // we don't specify an extra start state

	req.num_planning_attempts = p.get<uint>("num_planning_attempts");
	req.max_velocity_scaling_factor = p.get<double>("max_velocity_scaling_factor");
	req.max_acceleration_scaling_factor = p.get<double>("max_acceleration_scaling_factor");
	req.workspace_parameters = p.get<moveit_msgs::WorkspaceParameters>("workspace_parameters");
}

bool PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                           const planning_scene::PlanningSceneConstPtr& to, const moveit::core::JointModelGroup* jmg,
                           double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::Constraints& path_constraints) {
	const auto& props = properties();
	moveit_msgs::MotionPlanRequest req;
	initMotionPlanRequest(req, props, jmg, timeout);

	req.goal_constraints.resize(1);
	req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(to->getCurrentState(), jmg,
	                                                                          props.get<double>("goal_joint_tolerance"));
	req.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse res;
	bool success = planner_->generatePlan(from, req, res);
	result = res.trajectory_;
	return success;
}

bool PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                           const Eigen::Isometry3d& target_eigen, const moveit::core::JointModelGroup* jmg,
                           double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::Constraints& path_constraints) {
	const auto& props = properties();
	moveit_msgs::MotionPlanRequest req;
	initMotionPlanRequest(req, props, jmg, timeout);

	geometry_msgs::PoseStamped target;
	target.header.frame_id = from->getPlanningFrame();
	tf::poseEigenToMsg(target_eigen, target.pose);

	req.goal_constraints.resize(1);
	req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
	    link.getName(), target, props.get<double>("goal_position_tolerance"),
	    props.get<double>("goal_orientation_tolerance"));
	req.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse res;
	bool success = planner_->generatePlan(from, req, res);
	result = res.trajectory_;
	return success;
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
