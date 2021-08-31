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

#include "stages.h"
#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit_msgs/PlanningScene.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

PYBIND11_SMART_HOLDER_TYPE_CASTERS(ModifyPlanningScene)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(CurrentState)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(FixedState)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(ComputeIK)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(MoveTo)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(MoveRelative)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(Connect)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(FixCollisionObjects)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(GenerateGraspPose)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(GeneratePose)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(Pick)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(Place)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(SimpleGrasp)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(SimpleUnGrasp)

namespace moveit {
namespace python {

namespace {

/// extract from python argument a vector<T>, where arg maybe a single T or a list of Ts
template <typename T>
std::vector<T> elementOrList(const py::object& arg) {
	try {
		return std::vector<T>{ arg.cast<T>() };
	} catch (const py::cast_error&) {
		return arg.cast<std::vector<T>>();
	}
}

}  // anonymous namespace

void export_stages(pybind11::module& m) {
	// clang-format off
	properties::class_<ModifyPlanningScene, Stage>(m, "ModifyPlanningScene", R"pbdoc(
			Allows modification of the planning scene.
			This stage takes the incoming planning scene and applies previously scheduled changes to it, for example:
				- Modify allowed collision matrix, enabling or disabling collision pairs.
				- Attach or detach objects to robot links.
				- Spawn or remove objects.
		)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("modify planning scene"), R"pbdoc(
			Args:
				Name (str): Name of the stage.

			Returns:
				None
		)pbdoc")
		.def("attachObject", &ModifyPlanningScene::attachObject, R"pbdoc(

			Args:
				Name (str): Name of the object.
				Link (str): Name of the link, to which
							the object should be attached.

			Returns:
				None

		)pbdoc")
		.def("detachObject", &ModifyPlanningScene::detachObject, R"pbdoc(
			Detach an object from a robot link.
		)pbdoc")
		.def("attachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link, bool attach) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, attach);
		}, py::arg("names"), py::arg("attach_link"), py::arg("attach") = true, R"pbdoc(
			Attach multiple objects to a robot link.
		)pbdoc")
		.def("detachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, false);
		}, py::arg("names"), py::arg("attach_link"), R"pbdoc(
			Detach multiple objects from a robot link.
		)pbdoc")
		.def("allowCollisions", [](ModifyPlanningScene& self,
	        const py::object& first, const py::object& second, bool enable_collision) {
			self.allowCollisions(elementOrList<std::string>(first), elementOrList<std::string>(second), enable_collision);
		}, py::arg("first"), py::arg("second"), py::arg("enable_collision") = true, R"pbdoc(
			Allow or disable collisions between links and objects.
		)pbdoc");
	// clang-format on

	properties::class_<CurrentState, Stage>(m, "CurrentState", R"pbdoc(
			Fetch the current PlanningScene state via get_planning_scene service.
		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("current state"),
	         "class CurrentState(self, name)");

	properties::class_<FixedState, Stage>(m, "FixedState", R"pbdoc(
			Spawn a pre - defined PlanningScene state.
		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("fixed state"));

#if 0
		.def("setState", [](FixedState& stage, const moveit_msg::PlanningScene& scene_msg) {
			// TODO: How to initialize the PlanningScene?
			planning_scene::PlanningScenePtr scene;
			scene->setPlanningSceneMsg(scene_msg);
			stage.setState(scene);
		})
#endif
	;

	properties::class_<ComputeIK, Stage>(m, "ComputeIK", R"pbdoc(
			Wrapper for any pose generator stage to compute IK poses
			for a cartesian pose.

			The wrapper reads a target_pose from the interface state of
			solutions provided by the wrapped stage.
			This Cartesian pose (PoseStamped msg) is used as a goal pose
			for inverse kinematics.

			Usually, the end effector's parent link or the group's tip link
			is used as the IK frame, which should be moved to the goal frame.
			However, any other IK frame can be defined (which is linked to the tip of the group).

			Properties of the internally received InterfaceState can be
			forwarded to the newly generated, externally exposed InterfaceState.
		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: Specify which end effector should
			be used.
		)pbdoc")
	    .property<std::string>("group", R"pbdoc(
			str: Specify which planning group
			should be used.
		)pbdoc")
	    .property<std::string>("default_pose", R"pbdoc(
			str: Default joint pose of the active group
			(defines cost of IK).
		)pbdoc")
	    .property<uint32_t>("max_ik_solutions", R"pbdoc(
			int: Set the maximum number of inverse
			kinematic solutions thats should be generated.
		)pbdoc")
	    .property<bool>("ignore_collisions", R"pbdoc(
			bool: Specify if collisions with other members of
			the planning scene are allowed.
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("ik_frame", R"pbdoc(
			geometry_msgs/PoseStamped: Specify the frame with respect
			to which the inverse kinematics should be calculated.
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("target_pose", R"pbdoc(
			geometry_msgs/PoseStamped: Specify the pose on which
			the inverse kinematics should be
			calculated on. Since this property should almost always be set
			in the Interface State which is sent by the child,
			if possible, avoid setting it manually.
		)pbdoc")
	    // methods of base class py::class_ need to be called last!
	    .def(py::init<const std::string&, Stage::pointer&&>());

	properties::class_<MoveTo, PropagatingEitherWay, PyMoveTo<>>(m, "MoveTo")
	    .property<std::string>("group")
	    .property<geometry_msgs::PoseStamped>("ik_frame")
	    .property<moveit_msgs::Constraints>("path_constraints")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	    .def("setGoal", py::overload_cast<const geometry_msgs::PoseStamped&>(&MoveTo::setGoal))
	    .def("setGoal", py::overload_cast<const geometry_msgs::PointStamped&>(&MoveTo::setGoal))
	    .def("setGoal", py::overload_cast<const moveit_msgs::RobotState&>(&MoveTo::setGoal))
	    .def("setGoal", py::overload_cast<const std::map<std::string, double>&>(&MoveTo::setGoal))
	    .def("setGoal", py::overload_cast<const std::string&>(&MoveTo::setGoal));

	properties::class_<MoveRelative, PropagatingEitherWay, PyMoveRelative<>>(m, "MoveRelative", R"pbdoc(
			Perform a Cartesian motion relative to some link .
		)pbdoc")
	    .property<std::string>("group")
	    .property<geometry_msgs::PoseStamped>("ik_frame")
	    .property<double>("min_distance")
	    .property<double>("max_distance")
	    .property<moveit_msgs::Constraints>("path_constraints", R"pbdoc(
			These are the path constraints.
		)pbdoc")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	    .def("setDirection", py::overload_cast<const geometry_msgs::TwistStamped&>(&MoveRelative::setDirection),
	         R"pbdoc(
			Perform twist motion on specified link.
		)pbdoc")
	    .def("setDirection", py::overload_cast<const geometry_msgs::Vector3Stamped&>(&MoveRelative::setDirection),
	         R"pbdoc(
			Translate link along given direction.
		)pbdoc")
	    .def("setDirection", py::overload_cast<const std::map<std::string, double>&>(&MoveRelative::setDirection),
	         R"pbdoc(
			Move specified joint variables by given amount.
		)pbdoc");

	py::enum_<stages::Connect::MergeMode>(m, "MergeMode")
	    .value("SEQUENTIAL", stages::Connect::MergeMode::SEQUENTIAL)
	    .value("WAYPOINTS", stages::Connect::MergeMode::WAYPOINTS);
	PropertyConverter<stages::Connect::MergeMode>();

	properties::class_<Connect, Stage>(m, "Connect", R"pbdoc(
			Connect arbitrary InterfaceStates by motion planning

			The states may differ in various planning groups.
			To connect both states, the planners provided for
			individual sub groups are applied in the specified order.
			Each planner only plan for joints within the corresponding
			planning group. Finally, an attempt is made to merge the
			sub trajectories of individual planning results.
			If this fails, the sequential planning result is returned.
		)pbdoc")
	    .def(py::init<const std::string&, const Connect::GroupPlannerVector&>(),
	         py::arg("name") = std::string("connect"), py::arg("planners"));

	properties::class_<FixCollisionObjects, Stage>(m, "FixCollisionObjects", R"pbdoc(
			Test for collisions and find a correction for applicable objects.
			Move the objects out of the way along the correction direction.
		)pbdoc")
	    .property<double>("max_penetration", R"pbdoc(
			Cutoff length up to which collision objects get fixed.
		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("fix collisions"));

	properties::class_<GenerateGraspPose, MonitoringGenerator>(m, "GenerateGraspPose")
	    .property<std::string>("object")
	    .property<std::string>("eef")
	    .property<std::string>("pregrasp")
	    .property<std::string>("grasp")
	    .property<double>("angle_delta")
	    .def(py::init<const std::string&>());

	properties::class_<GeneratePose, MonitoringGenerator>(m, "GeneratePose")
	    .property<geometry_msgs::PoseStamped>("pose")
	    .def(py::init<const std::string&>());

	properties::class_<Pick, Stage>(m, "Pick", R"pbdoc(
			Specialization of PickPlaceBase to realize picking
		)pbdoc")
	    .property<std::string>("object")
	    .property<std::string>("eef")
	    .property<std::string>("eef_frame")
	    .property<std::string>("eef_group")
	    .property<std::string>("eef_parent_group")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("grasp_generator"),
	         py::arg("name") = std::string("pick"))
	    .def("setApproachMotion", &Pick::setApproachMotion)
	    .def("setLiftMotion",
	         py::overload_cast<const geometry_msgs::TwistStamped&, double, double>(&Pick::setLiftMotion))
	    .def("setLiftMotion", py::overload_cast<const std::map<std::string, double>&>(&Pick::setLiftMotion));

	properties::class_<Place, Stage>(m, "Place", R"pbdoc(
			Specialization of PickPlaceBase to realize placing
		)pbdoc")
	    .property<std::string>("object")
	    .property<std::string>("eef")
	    .property<std::string>("eef_frame")
	    .property<std::string>("eef_group")
	    .property<std::string>("eef_parent_group")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("place_generator"),
	         py::arg("name") = std::string("place"));

	properties::class_<SimpleGrasp, Stage>(m, "SimpleGrasp", R"pbdoc(
			Specialization of SimpleGraspBase to realize grasping
		)pbdoc")
	    .property<std::string>("eef")
	    .property<std::string>("object")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("pose_generator"),
	         py::arg("name") = std::string("grasp generator"))
	    .def<void (SimpleGrasp::*)(const geometry_msgs::PoseStamped&)>("setIKFrame", &SimpleGrasp::setIKFrame)
	    .def<void (SimpleGrasp::*)(const Eigen::Isometry3d&, const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame)
	    .def<void (SimpleGrasp::*)(const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame);

	properties::class_<SimpleUnGrasp, Stage>(m, "SimpleUnGrasp", R"pbdoc(
			Specialization of SimpleGraspBase to realize ungrasping
		)pbdoc")
	    .property<std::string>("eef")
	    .property<std::string>("object")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("pose_generator"),
	         py::arg("name") = std::string("place generator"));
}
}  // namespace python
}  // namespace moveit
