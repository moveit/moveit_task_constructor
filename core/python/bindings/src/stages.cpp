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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <pybind11/stl.h>
#include <py_binding_tools/ros_msg_typecasters.h>

namespace py = pybind11;
using namespace py::literals;
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
PYBIND11_SMART_HOLDER_TYPE_CASTERS(GeneratePlacePose)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(GeneratePose)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(Pick)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(Place)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(SimpleGraspBase)
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
	properties::class_<ModifyPlanningScene, Stage>(m, "ModifyPlanningScene", R"(
		Apply modifications to the PlanningScene w/o moving the robot

		This stage takes the incoming planning scene and applies previously scheduled changes to it, for example:

		* Modify allowed collision matrix, enabling or disabling collision pairs
		* Attach or detach objects to robot links
		* Add or remove objects

		For an example, see :ref:`How-To-Guides <subsubsec-howto-modify-planning-scene>`.
		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("modify planning scene"))
		.def("attachObject", &ModifyPlanningScene::attachObject, "Attach an object to a robot link", "name"_a, "link"_a)
		.def("detachObject", &ModifyPlanningScene::detachObject, "Detach an object from a robot link", "name"_a, "link"_a)
		.def("attachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link, bool attach) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, attach);
		}, "Attach multiple objects to a robot link", "names"_a, "attach_link"_a, "attach"_a = true)
		.def("detachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, false);
		}, "Detach multiple objects from a robot link", "names"_a, "attach_link"_a)
		.def("allowCollisions", [](ModifyPlanningScene& self, const std::string& object, bool allow) {self.allowCollisions(object, allow);},
			"Allow or disable all collisions involving the given object", "object"_a, "enable_collision"_a = true)
		.def("allowCollisions", [](ModifyPlanningScene& self,
	        const py::object& first, const py::object& second, bool enable_collision) {
			self.allowCollisions(elementOrList<std::string>(first), elementOrList<std::string>(second), enable_collision);
		}, "Allow or disable collisions between links and objects", "first"_a, "second"_a, "enable_collision"_a = true)
		.def("addObject", &ModifyPlanningScene::addObject, R"(
			Add a CollisionObject_ to the planning scene

			.. _CollisionObject: https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html

		)", "collision_object"_a)
		.def("removeObject", &ModifyPlanningScene::removeObject,
			"Remove a CollisionObject_ from the planning scene", "name"_a)
		;

	properties::class_<CurrentState, Stage>(m, "CurrentState", R"(
			Fetch the current PlanningScene / real robot state via the ``get_planning_scene`` service.
			Usually, this stage should be used *once* at the beginning of your pipeline to start from the current state.

			.. literalinclude:: ./../../../demo/scripts/current_state.py
				:language: python

		)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("current state"));

	properties::class_<FixedState, Stage>(m, "FixedState", R"(
			Spawn a pre-defined PlanningScene state.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-fixed-state>`
			for an implementation of a task hierarchy that makes use of the
			``FixedState`` stage.
		)")
		.def("setState", &FixedState::setState, R"(
			Use a planning scene pointer to specify which state the Fixed State
			stage should have.
		)", "scene"_a)
	    .def(py::init<const std::string&>(), "name"_a = std::string("fixed state"));

#if 0
		.def("setState", [](FixedState& stage, const moveit_msg::PlanningScene& scene_msg) {
			// TODO: How to initialize the PlanningScene?
			planning_scene::PlanningScenePtr scene;
			scene->setPlanningSceneMsg(scene_msg);
			stage.setState(scene);
		})
#endif
	;

	properties::class_<ComputeIK, Stage>(m, "ComputeIK", R"(
			Wrapper for any pose generator stage to compute the inverse
			kinematics for a pose in Cartesian space.

			The wrapper reads a ``target_pose`` from the interface state of
			solutions provided by the wrapped stage. This cartesian pose
			(``PoseStamped`` msg) is used as a goal pose for inverse
			kinematics.

			Usually, the end effector's parent link or the group's tip link
			is used as the inverse kinematics frame, which should be
			moved to the goal frame. However, any other inverse kinematics
			frame can be defined (which is linked to the tip of the group).

			Properties of the internally received ``InterfaceState`` can be
			forwarded to the newly generated, externally exposed ``InterfaceState``.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-compute-ik>`
			for an implementation of a task hierarchy that makes use of the
			``ComputeIK`` stage.
		)")
	    .property<std::string>("eef", R"(
			str: Specify which end effector of the active planning group
			should be used.
		)")
	    .property<std::string>("group", R"(
			str: Specify which planning group
			should be used.
		)")
	    .property<std::string>("default_pose", R"(
			str: Default joint pose of the active group
			(defines cost of the inverse kinematics).
		)")
	    .property<uint32_t>("max_ik_solutions", R"(
			int: Set the maximum number of inverse
			kinematic solutions thats should be generated.
		)")
	    .property<uint32_t>("max_ik_solutions", "uint: max number of solutions to return")
	    .property<bool>("ignore_collisions", R"(
			bool: Specify if collisions with other members of
			the planning scene are allowed.
		)")
	    .property<double>("min_solution_distance", "reject solution that are closer than this to previously found solutions")
	    .property<moveit_msgs::msg::Constraints>("constraints", "additional constraints to obey")
	    .property<geometry_msgs::msg::PoseStamped>("ik_frame", R"(
			PoseStamped_: Specify the frame with respect
			to which the inverse kinematics
			should be calculated.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
	    .property<geometry_msgs::msg::PoseStamped>("target_pose", R"(
			PoseStamped_: Specify the pose on which
			the inverse kinematics should be
			calculated on. Since this property should
			almost always be set
			in the Interface State which is sent by the child,
			if possible, avoid setting it manually.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
	    // methods of base class py::class_ need to be called last!
	    .def(py::init<const std::string&, Stage::pointer&&>(), "name"_a, "stage"_a);

	properties::class_<MoveTo, PropagatingEitherWay, PyMoveTo<>>(m, "MoveTo", R"(
			Compute a trajectory between the robot state from the
			interface state of the preceeding stage and a specified
			goal.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-move-to>`
			for an implementation of a task hierarchy that makes use of the
			``MoveTo`` stage.
		)")
	    .property<std::string>("group", R"(
			str: Planning group which should be utilized for planning and execution.
		)")
	    .property<geometry_msgs::msg::PoseStamped>("ik_frame", R"(
			PoseStamped_: IK reference frame for the goal pose

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html

		)")
	    .property<moveit_msgs::msg::Constraints>("path_constraints", R"(
			Constraints_: Set path constraints via the corresponding moveit message type

			.. _Constraints: https://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
		)")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>(), "name"_a, "planner"_a)
	    .def("setGoal", py::overload_cast<const geometry_msgs::msg::PoseStamped&>(&MoveTo::setGoal), R"(
			Move link to a given PoseStamped_

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)", "goal"_a)
	    .def("setGoal", py::overload_cast<const geometry_msgs::msg::PointStamped&>(&MoveTo::setGoal), R"(
			Move link to given PointStamped_, keeping current orientation

			.. _PointStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PointStamped.html
		)", "goal"_a)
	    .def("setGoal", py::overload_cast<const moveit_msgs::msg::RobotState&>(&MoveTo::setGoal), R"(
			Move joints specified in RobotState_ to their target values

			.. _RobotState: https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/RobotState.html
		)", "goal"_a)
	    .def("setGoal", py::overload_cast<const std::map<std::string, double>&>(&MoveTo::setGoal), R"(
			Move joints by name to their mapped target value provided by dict goal argument
		)", "goal"_a)
	    .def("setGoal", py::overload_cast<const std::string&>(&MoveTo::setGoal), R"(
			Move joint model group to given named pose provided as a str argument
		)", "goal"_a);

	properties::class_<MoveRelative, PropagatingEitherWay, PyMoveRelative<>>(m, "MoveRelative", R"(
			Perform a Cartesian motion relative to some link.
			Take a look at the :ref:`How-To-Guides <subsubsec-howto-move-relative>`
			for an implementation of a task hierarchy that makes use of the
			``MoveRelative`` stage.
			To implement your own propagtor logic on top of the ``MoveRelative`` class' functionality,
			you may derive from the stage. Take a look at the corresponding
			:ref:`How-To-Guide <subsubsec-howto-move-relative>`.
		)")
	    .property<std::string>("group", R"(
			str: Planning group which should be utilized for planning and execution.
		)")
	    .property<geometry_msgs::msg::PoseStamped>("ik_frame", R"(
			PoseStamped_: IK reference frame for the goal pose.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
	    .property<double>("min_distance", "float: Set the minimum distance to move")
	    .property<double>("max_distance", "float: Set the maximum distance to move")
	    .property<moveit_msgs::msg::Constraints>("path_constraints", R"(
			Constraints_: These are the path constraints.

			.. _Constraints: https://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
		)")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>(), "name"_a, "planner"_a)
	    .def("setDirection", py::overload_cast<const geometry_msgs::msg::TwistStamped&>(&MoveRelative::setDirection), R"(
			Perform twist motion on specified link.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)", "twist"_a)
	    .def("setDirection", py::overload_cast<const geometry_msgs::msg::Vector3Stamped&>(&MoveRelative::setDirection), R"(
			Translate link along given direction.

			.. _Vector3Stamped: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3Stamped.html
		)", "direction"_a)
	    .def("setDirection", py::overload_cast<const std::map<std::string, double>&>(&MoveRelative::setDirection), R"(
			Move specified joint variables by given amount.
		)", "joint_deltas"_a);

	py::enum_<stages::Connect::MergeMode>(m, "MergeMode", R"(
			Define the merge strategy to use when performing planning operations
			with e.g. the connect stage.
		)")
	    .value("SEQUENTIAL", stages::Connect::MergeMode::SEQUENTIAL, "Store sequential trajectories")
	    .value("WAYPOINTS", stages::Connect::MergeMode::WAYPOINTS, "Join trajectories by their waypoints");
	PropertyConverter<stages::Connect::MergeMode>();

	properties::class_<Connect, Stage>(m, "Connect", R"(
			Connect arbitrary InterfaceStates by motion planning.
			You can specify the planning groups and the planners you
			want to utilize.

			The states may differ in various planning groups.
			To connect both states, the planners provided for
			individual sub groups are applied in the specified order.
			Each planner only plan for joints within the corresponding
			planning group. Finally, an attempt is made to merge the
			sub trajectories of individual planning results.
			If this fails, the sequential planning result is returned.

			For an example, see :ref:`How-To-Guides <subsubsec-howto-connect>`.
		)")
	    .property<stages::Connect::MergeMode>("merge_mode", "Defines the merge strategy to use")
	    .property<double>("max_distance", "maximally accepted distance between end and goal sate")
	    .def(py::init<const std::string&, const Connect::GroupPlannerVector&>(),
	         "name"_a = std::string("connect"), "planners"_a);

	properties::class_<FixCollisionObjects, Stage>(m, "FixCollisionObjects", R"(
			Test for collisions and find a correction for applicable objects.
			Move the objects out of the way along the correction direction.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-fix-collision-objects>`
			for an implementation of a task hierarchy that makes use of the
			``FixCollisionObjects`` stage.
		)")
	    .property<double>("max_penetration", R"(
			float: Cutoff length up to which collision objects get fixed.
		)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("fix collisions"));

	properties::class_<GeneratePlacePose, MonitoringGenerator>(m, "GeneratePlacePose", R"(
			GeneratePlacePose stage derives from monitoring generator and generates poses
			for the place pipeline. Notice that whilst GenerateGraspPose spawns poses with an
			``angle_delta`` intervall, GeneratePlacePose samples a fixed amount, which is dependent
			on the objects shape.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-generate-place-pose>`
			for a snippet that demonstrates usage of the `GeneratePlacePose` stage.
		)")
		.property<std::string>("object", R"(
			str: Name of the object in the planning scene, attached to the robot which should be placed
		)")
		.property<std::string>("eef", "str: Name of the end effector that should be used for grasping")
		.property<geometry_msgs::msg::PoseStamped>("pose", R"(
			PoseStamped_: The pose where the object should be placed, i.e. states should be sampled

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("Generate Place Pose"));


	properties::class_<GenerateGraspPose, MonitoringGenerator>(m, "GenerateGraspPose", R"(
			GenerateGraspPose stage derives from monitoring generator and can
			be used to generate poses for grasping. Set the desired attributes
			of the grasp using the stages properties.
			Take a look at the :ref:`How-To-Guides <subsubsec-howto-generate-grasp-pose>`
			for a snippet that demonstrates usage of the `GenerateGraspPose` stage.
		)")
	    .property<std::string>("object", R"(
			str: Name of the Object in the planning scene, which should be grasped
		)")
	    .property<std::string>("eef", R"(
			str: Name of the end effector that should be used for grasping
		)")
	    .property<std::string>("pregrasp", "str: Name of the pre-grasp pose")
	    .property<std::string>("grasp", "str: Name of the grasp pose")
	    .property<double>("angle_delta", R"(
			float: Angular step distance in rad with which positions around the object are sampled.
		)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("Generate Grasp Pose"));

	properties::class_<GeneratePose, MonitoringGenerator>(m, "GeneratePose", R"(
			Monitoring generator stage which can be used to generate a pose, based on solutions provided
			by the monitored stage.

			Take a look at the :ref:`How-To-Guides <subsubsec-howto-fix-collision-objects>`
			for an implementation of a task hierarchy that makes use of the
			``GeneratePose`` stage.
		)")
	    .property<geometry_msgs::msg::PoseStamped>("pose", R"(
			PoseStamped_: Set the pose, which should be spawned on each new solution of the monitored stage.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
	    .def(py::init<const std::string&>(), "name"_a);

	properties::class_<Pick, SerialContainer>(m, "Pick", R"(
			The Pick stage is a specialization of the PickPlaceBase class, which
			wraps the pipeline to pick or place an object with a given end effector.

			Picking consist of the following sub stages:

				- Linearly approaching the object along an approach direction/twist "grasp" end effector posture
				- Attach the object
				- Lift along a given direction/twist

			The end effector postures corresponding to pre-grasp and grasp as well
			as the end effector's cartesian pose needs to be provided by an external
			grasp stage.

			Take a look at the :ref:`Pick and Place Tutorial <subsec-tut-pick-place>` for an in-depth look,
			as well as the :ref:`How-To Guide <subsubsec-howto-pick>` for a minimal implementation
			of a task hierarchy that makes use of the
			``Pick`` stage.
		)")
	    .property<std::string>("object", "str: Name of object to pick")
	    .property<std::string>("eef", "str: The End effector name")
	    .property<std::string>("eef_frame", "str: Name of the end effector frame")
	    .property<std::string>("eef_group", "str: Joint model group of the end effector")
	    .property<std::string>("eef_parent_group", "str: Joint model group of the eef's parent")
	    .def(py::init<Stage::pointer&&, const std::string&>(), "grasp_generator"_a,
	         "name"_a = std::string("pick"))
	    .def("setApproachMotion", &Pick::setApproachMotion, R"(
			The approaching motion towards the grasping state is represented
			by a twist message.
			Additionally specify the minimum and maximum allowed distances to travel.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)", "motion"_a, "min_distance"_a, "max_distance"_a)
	    .def("setLiftMotion", py::overload_cast<const geometry_msgs::msg::TwistStamped&, double, double>(&Pick::setLiftMotion), R"(
			The lifting motion away from the grasping state is represented by a twist message.
			Additionally specify the minimum and maximum allowed distances to travel.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)", "motion"_a, "min_distance"_a, "max_distance"_a)
	    .def("setLiftMotion", py::overload_cast<const std::map<std::string, double>&>(&Pick::setLiftMotion), R"(
			The lifting motion away from the grasping state is represented by its destination as joint-value pairs
		)", "place"_a);

	properties::class_<Place, SerialContainer>(m, "Place", R"(
			The Place stage is a specialization of the PickPlaceBase class, which
			wraps the pipeline to pick or place an object with a given end effector.

			Placing consist of the inverse order of stages:

				- Place down along a given direction
				- Detach the object
				- Linearly retract end effector

			The end effector postures corresponding to pre-grasp and grasp as well
			as the end effector's Cartesian pose needs to be provided by an external
			grasp stage.

			Take a look at the :ref:`Pick and Place Tutorial <subsec-tut-pick-place>` for an in-depth look,
			as well as the :ref:`How-To Guide <subsubsec-howto-place>` for a minimal implementation
			of a task hierarchy that makes use of the
			``Place`` stage.
		)")
	    .property<std::string>("object", "str: Name of object to pick")
	    .property<std::string>("eef", "str: The End effector name")
	    .property<std::string>("eef_frame", "str: Name of the end effector frame")
	    .property<std::string>("eef_group", "str: Joint model group of the end effector")
	    .property<std::string>("eef_parent_group", "str: Joint model group of the eef's parent")
    	.def("setRetractMotion", &Place::setRetractMotion, R"(
			The retract motion towards the final state is represented
			by a Twist_ message. Additionally specify the minimum and
			maximum allowed distances to travel.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)", "motion"_a, "min_distance"_a, "max_distance"_a)
	    .def("setPlaceMotion", py::overload_cast<const geometry_msgs::msg::TwistStamped&, double, double>(&Place::setPlaceMotion), R"(
			The object-placing motion towards the final state is represented by a twist message.
			Additionally specify the minimum and maximum allowed distances to travel.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)", "motion"_a, "min_distance"_a, "max_distance"_a )
	    .def("setPlaceMotion", py::overload_cast<const std::map<std::string, double>&>(&Place::setPlaceMotion), R"(
			The placing motion to the final state is represented by its destination as joint-value pairs
		)", "joints"_a )
	    .def(py::init<Stage::pointer&&, const std::string&>(), "place_generator"_a,
	         "name"_a = std::string("place"));

	properties::class_<SimpleGraspBase, SerialContainer>(m, "SimpleGraspBase", "Abstract base class for grasping and releasing")
		.property<std::string>("eef", "str: The end effector of the robot")
		.property<std::string>("object", "str: The object to grasp (Must be present in the planning scene)")
		.property<geometry_msgs::msg::PoseStamped>("ik_frame", R"(
			PoseStamped_: Set the frame for which the inverse kinematics is calculated
			with respect to each pose generated by the pose_generator.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)")
		.def<void (SimpleGraspBase::*)(const geometry_msgs::msg::PoseStamped&)>("setIKFrame", &SimpleGraspBase::setIKFrame, R"(
			Set the frame as a PoseStamped_ for which the inverse kinematics are calculated with respect to
			each pose generated by the pose_generator.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)", "transform"_a)
		.def<void (SimpleGraspBase::*)(const Eigen::Isometry3d&, const std::string&)>("setIKFrame", &SimpleGraspBase::setIKFrame, R"(
			Set the frame as a PoseStamped_ for which the inverse kinematics are calculated
			with respect to each pose generated by the pose_generator.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)", "pose"_a, "link"_a)
		.def<void (SimpleGraspBase::*)(const std::string&)>("setIKFrame", &SimpleGraspBase::setIKFrame, R"(
			Set the frame for which the inverse kinematics are calculated
			with respect to each pose generated by the pose_generator.
		)", "link"_a)
		.def("setMaxIKSolutions", &SimpleGraspBase::setMaxIKSolutions, R"(
			Set the maximum number of inverse kinematics solutions that should be computed.
		)", "max_ik_solutions"_a)
		;
	properties::class_<SimpleGrasp, SimpleGraspBase>(m, "SimpleGrasp", R"(
			Specialization of SimpleGraspBase to realize grasping.

			Take a look at the :ref:`Pick and Place Tutorial <subsec-tut-pick-place>` for an in-depth look,
			as well as the :ref:`How-To Guide <subsubsec-howto-simplegrasp>` for a minimal implementation
			of a task hierarchy that makes use of the
			``SimpleGrasp`` stage.
		)")
	    .def(py::init<Stage::pointer&&, const std::string&>(), "pose_generator"_a,
	         "name"_a = std::string("grasp"));

	properties::class_<SimpleUnGrasp, SimpleGraspBase>(m, "SimpleUnGrasp", R"(
			Specialization of SimpleGraspBase to realize ungrasping

			Take a look at the :ref:`Pick and Place Tutorial <subsec-tut-pick-place>` for an in-depth look,
			as well as the :ref:`How-To Guide <subsubsec-howto-simplegrasp>` for a minimal implementation
			of a task hierarchy that makes use of the
			``SimpleUnGrasp`` stage.
		)")
		.property<std::string>("pregrasp", "str: Name of the pre-grasp pose")
		.property<std::string>("grasp", "str: Name of the grasp pose")
		.def(py::init<Stage::pointer&&, const std::string&>(), "pose_generator"_a,
		     "name"_a = std::string("ungrasp"));
}
}  // namespace python
}  // namespace moveit
