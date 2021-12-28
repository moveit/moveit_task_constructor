/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
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

#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_msgs/WorkspaceParameters.h>

namespace py = pybind11;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::solvers;

PYBIND11_SMART_HOLDER_TYPE_CASTERS(PlannerInterface)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(PipelinePlanner)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(JointInterpolationPlanner)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(CartesianPath)

namespace moveit {
namespace python {

void export_solvers(py::module& m) {
	properties::class_<PlannerInterface>(m, "PlannerInterface")
	    .property<double>("max_velocity_scaling_factor")
	    .property<double>("max_acceleration_scaling_factor")
	    .def_property_readonly("properties", py::overload_cast<>(&PlannerInterface::properties),
	                           py::return_value_policy::reference_internal);

	properties::class_<PipelinePlanner, PlannerInterface>(m, "PipelinePlanner", R"pbdoc(
			PipelinePlanner()

			Use MoveIt's PlanningPipeline to plan a trajectory between to scenes.

			::

				# create and configure a planner instance
				pipelinePlanner = core.PipelinePlanner()
				pipelinePlanner.planner = 'PRMkConfigDefault'
				pipelinePlanner.num_planning_attempts = 10

				# specify planning group
				group = "panda_arm"

				# create a task
				task = core.Task()

				# get the current robot state
				currentState = stages.CurrentState("current state")
				task.add(currentState)

				# moveTo named posture, using the pipeline planner interplation
				move = stages.MoveTo("moveTo ready", pipelinePlanner)
				move.group = group
				move.setGoal("extended")
				task.add(move)

		)pbdoc")
	    .property<std::string>("planner", R"pbdoc(
			str: Planner ID.
		)pbdoc")
	    .property<uint>("num_planning_attempts", R"pbdoc(
			uint: Number of planning attempts.
		)pbdoc")
	    .property<moveit_msgs::WorkspaceParameters>("workspace_parameters", R"pbdoc(
				WorkspaceParameters_: Workspace_parameters for the planning algorithm

			.. _WorkspaceParameters: https://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/WorkspaceParameters.html
		)pbdoc")
	    .property<double>("goal_joint_tolerance", R"pbdoc(
			float: Tolerance for reaching joint goals.
		)pbdoc")
	    .property<double>("goal_position_tolerance", R"pbdoc(
			float: Tolerance for reaching position goals.
		)pbdoc")
	    .property<double>("goal_orientation_tolerance", R"pbdoc(
			float: Tolerance for reaching orientation goals.
		)pbdoc")
	    .property<bool>("display_motion_plans", R"pbdoc(
			bool: Publish generated solutions via a topic.
		)pbdoc")
	    .property<bool>("publish_planning_requests", R"pbdoc(
			bool: Publish motion planning requests via a topic.
		)pbdoc")
	    .def(py::init<>());

	properties::class_<JointInterpolationPlanner, PlannerInterface>(m, "JointInterpolationPlanner", R"pbdoc(
			JointInterpolationPlanner()

			Interpolate a trajectory between states in joint space.
			Fails if direct joint space interpolation fails.

			::

				from moveit.task_constructor import core

				# Instantiate joint interpolation planner
				jointPlanner = core.JointInterpolationPlanner()
				jointPlanner.max_step = 0.1
		)pbdoc")
	    .property<double>("max_step", R"pbdoc(
			float: Prevent large movements by specifying the maximum step distance
				in joint space.
		)pbdoc")
	    .def(py::init<>());

	properties::class_<CartesianPath, PlannerInterface>(m, "CartesianPath", R"pbdoc(
			CartesianPath()

			The planner uses MoveIt's ``computeCartesianPath()`` to
			generate a straigh-line path between two scenes.

			::

				from moveit.task_constructor import core

				# Instantiate cartesian path planner
				cartesianPlanner = core.CartesianPath()
				cartesianPlanner.step_size = 0.01
				cartesianPlanner.jump_threshold = 0.0  # effectively disable jump threshold.
		)pbdoc")
	    .property<double>("step_size", R"pbdoc(
			float: Specify the path interpolation resolution by adjusting the
				step size between consecutive waypoints with this parameter.
		)pbdoc")
	    .property<double>("jump_threshold", R"pbdoc(
			float: Prevent jumps in inverse kinematic solutions by adjusting the
				acceptable fraction of mean joint motion per step.
		)pbdoc")
	    .property<double>("min_fraction", R"pbdoc(
			float: Lower threshold of minimum motion per step required for
				success in planning.
		)pbdoc")
	    .def(py::init<>());
}
}  // namespace python
}  // namespace moveit
