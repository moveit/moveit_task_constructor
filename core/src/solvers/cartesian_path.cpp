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
   Desc:    generate and validate a straight-line Cartesian path
*/

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#if MOVEIT_MASTER
#include <moveit/robot_state/cartesian_interpolator.h>
#endif

namespace moveit {
namespace task_constructor {
namespace solvers {

CartesianPath::CartesianPath() {
	auto& p = properties();
	p.declare<double>("step_size", 0.01, "step size between consecutive waypoints");
	p.declare<double>("jump_threshold", 1.5, "acceptable fraction of mean joint motion per step");
	p.declare<double>("min_fraction", 1.0, "fraction of motion required for success");
}

void CartesianPath::init(const core::RobotModelConstPtr& robot_model) {}

bool CartesianPath::plan(const planning_scene::PlanningSceneConstPtr& from,
                         const planning_scene::PlanningSceneConstPtr& to, const moveit::core::JointModelGroup* jmg,
                         double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                         const moveit_msgs::Constraints& path_constraints) {
	const moveit::core::LinkModel* link = jmg->getOnlyOneEndEffectorTip();
	if (!link) {
		ROS_WARN_STREAM("no unique tip for joint model group: " << jmg->getName());
		return false;
	}

	// reach pose of forward kinematics
	return plan(from, *link, to->getCurrentState().getGlobalLinkTransform(link), jmg, timeout, result, path_constraints);
}

bool CartesianPath::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                         const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg, double timeout,
                         robot_trajectory::RobotTrajectoryPtr& result,
                         const moveit_msgs::Constraints& path_constraints) {
	const auto& props = properties();
	planning_scene::PlanningScenePtr sandbox_scene = from->diff();

	kinematic_constraints::KinematicConstraintSet kcs(sandbox_scene->getRobotModel());
	kcs.add(path_constraints, sandbox_scene->getTransforms());

	auto is_valid = [&sandbox_scene, &kcs](moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
	                                       const double* joint_positions) {
		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		return !sandbox_scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()) &&
		       kcs.decide(*state).satisfied;
	};

	std::vector<moveit::core::RobotStatePtr> trajectory;
#if MOVEIT_MASTER
	double achieved_fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
	    &(sandbox_scene->getCurrentStateNonConst()), jmg, trajectory, &link, target, true,
	    moveit::core::MaxEEFStep(props.get<double>("step_size")),
	    moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid);
#else
	double achieved_fraction = sandbox_scene->getCurrentStateNonConst().computeCartesianPath(
	    jmg, trajectory, &link, target, true, props.get<double>("step_size"), props.get<double>("jump_threshold"),
	    is_valid);
#endif

	if (!trajectory.empty()) {
		result.reset(new robot_trajectory::RobotTrajectory(sandbox_scene->getRobotModel(), jmg));
		for (const auto& waypoint : trajectory)
			result->addSuffixWayPoint(waypoint, 0.0);

		trajectory_processing::IterativeParabolicTimeParameterization timing;
		timing.computeTimeStamps(*result, props.get<double>("max_velocity_scaling_factor"),
		                         props.get<double>("max_acceleration_scaling_factor"));
	}

	return achieved_fraction >= props.get<double>("min_fraction");
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
