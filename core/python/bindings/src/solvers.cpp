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

	properties::class_<PipelinePlanner, PlannerInterface>(m, "PipelinePlanner")
	    .property<std::string>("planner")
	    .property<uint>("num_planning_attempts")
	    .property<moveit_msgs::WorkspaceParameters>("workspace_parameters")
	    .property<double>("goal_joint_tolerance")
	    .property<double>("goal_position_tolerance")
	    .property<double>("goal_orientation_tolerance")
	    .property<bool>("display_motion_plans")
	    .property<bool>("publish_planning_requests")
	    .def(py::init<>());

	properties::class_<JointInterpolationPlanner, PlannerInterface>(m, "JointInterpolationPlanner")
	    .property<double>("max_step")
	    .def(py::init<>());

	properties::class_<CartesianPath, PlannerInterface>(m, "CartesianPath")
	    .property<double>("step_size")
	    .property<double>("jump_threshold")
	    .property<double>("min_fraction")
	    .def(py::init<>());
}
}  // namespace python
}  // namespace moveit
