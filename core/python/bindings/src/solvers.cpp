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
#include <moveit/task_constructor/solvers/multi_planner.h>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include "utils.h"

namespace py = pybind11;
using namespace py::literals;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::solvers;

PYBIND11_SMART_HOLDER_TYPE_CASTERS(PlannerInterface)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(PipelinePlanner)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(JointInterpolationPlanner)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(CartesianPath)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(MultiPlanner)

namespace moveit {
namespace python {

void export_solvers(py::module& m) {
	properties::class_<PlannerInterface>(m, "PlannerInterface", "Abstract base class for planning algorithms")
	    .property<double>("max_velocity_scaling_factor", "float: Reduce the maximum velocity by scaling between (0,1]")
	    .property<double>("max_acceleration_scaling_factor",
	                      "float: Reduce the maximum acceleration by scaling between (0,1]")
	    .def_property_readonly("properties", py::overload_cast<>(&PlannerInterface::properties),
	                           py::return_value_policy::reference_internal, "Properties of the planner");

	properties::class_<PipelinePlanner, PlannerInterface>(m, "PipelinePlanner",
	                                                      R"(Plan using MoveIt's ``PlanningPipeline``
			::

				from moveit.task_constructor import core

				# Create and configure a planner instance
				pipelinePlanner = core.PipelinePlanner()
				pipelinePlanner.planner = 'PRMkConfigDefault'
				pipelinePlanner.num_planning_attempts = 10
			)")
	    .property<std::string>("planner", "str: Planner ID")
	    .property<uint>("num_planning_attempts", "int: Number of planning attempts")
	    .property<moveit_msgs::msg::WorkspaceParameters>(
	        "workspace_parameters",
	        ":moveit_msgs:`WorkspaceParameters`: Specifies workspace box to be used for Cartesian sampling")
	    .property<double>("goal_joint_tolerance", "float: Tolerance for reaching joint goals")
	    .property<double>("goal_position_tolerance", "float: Tolerance for reaching position goals")
	    .property<double>("goal_orientation_tolerance", "float: Tolerance for reaching orientation goals")
	    .def(py::init<const rclcpp::Node::SharedPtr&, const std::string&>(), "node"_a,
	         "pipeline"_a = std::string("ompl"));

	properties::class_<JointInterpolationPlanner, PlannerInterface>(
	    m, "JointInterpolationPlanner",
	    R"(Perform linear interpolation between joint space poses.
			Fails on collision along the interpolation path. There is no obstacle avoidance. ::

				from moveit.task_constructor import core

				# Instantiate joint-space interpolation planner
				jointPlanner = core.JointInterpolationPlanner()
				jointPlanner.max_step = 0.1
		)")
	    .property<double>("max_step", "float: Limit any (single) joint change between two waypoints to this amount")
	    .def(py::init<>());

	properties::class_<CartesianPath, PlannerInterface>(m, "CartesianPath", R"(
			Perform linear interpolation between Cartesian poses.
		 	Fails on collision along the interpolation path. There is no obstacle avoidance. ::

				from moveit.task_constructor import core

				# Instantiate Cartesian-space interpolation planner
				cartesianPlanner = core.CartesianPath()
				cartesianPlanner.step_size = 0.01
				cartesianPlanner.jump_threshold = 0.0  # effectively disable jump threshold.
		)")
	    .property<double>("step_size", "float: Limit the Cartesian displacement between consecutive waypoints "
	                                   "In contrast to joint-space interpolation, the Cartesian planner can also "
	                                   "succeed when only a fraction of the linear path was feasible.")
	    .property<double>(
	        "jump_threshold",
	        "float: Limit joint displacement between consecutive waypoints, thus preventing jumps in joint space. "
	        "This values specifies the fraction of mean acceptable joint motion per step.")
	    .property<double>("min_fraction", "float: Fraction of overall distance required to succeed.")
	    .def(py::init<>());

	properties::class_<MultiPlanner, PlannerInterface>(m, "MultiPlanner", R"(
			A meta planner that runs multiple alternative planners in sequence and returns the first found solution. ::

				from moveit.task_constructor import core

				# Instantiate MultiPlanner
				multiPlanner = core.MultiPlanner()
		)")
	    .def("__len__", &MultiPlanner::size)
	    .def("__getitem__", &get_item<MultiPlanner>)
	    .def(
	        "add",
	        [](MultiPlanner& self, const py::args& args) {
		        for (auto it = args.begin(), end = args.end(); it != end; ++it)
			        self.push_back(it->cast<PlannerInterfacePtr>());
	        },
	        "Insert one or more planners")
	    .def(
	        "clear", [](MultiPlanner& self) { self.clear(); }, "Remove all planners")
	    .def(py::init<>());
}
}  // namespace python
}  // namespace moveit
