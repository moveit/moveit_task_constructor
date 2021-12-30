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
PYBIND11_SMART_HOLDER_TYPE_CASTERS(GeneratePlacePose)
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
		ModifyPlanningScene(name)

		Allows modification of the planning scene.
		This stage takes the incoming planning scene and applies previously scheduled changes to it, for example:
			- Modify allowed collision matrix, enabling or disabling collision pairs.
			- Attach or detach objects to robot links.
			- Spawn or remove objects.

		Args:
			name (str): Name of the stage.

		.. literalinclude:: ./../../../demo/scripts/modify_planning_scene.py
			:language: python

		)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("modify planning scene"))
		.def("attachObject", &ModifyPlanningScene::attachObject, R"pbdoc(
			attachObject(name, link)

			Args:
				name (str): Name of the object
				link (str): Name of the link, to which
					the object should be attached.
			Returns:
				None
		)pbdoc")
		.def("detachObject", &ModifyPlanningScene::detachObject, R"pbdoc(
			detachObject(name, link)

			Detach an object from a robot link.

			Args:
				name (str): Object name that should be detached.
				link (str): Link name from which the object should be detached.
			Returns:
				None
		)pbdoc")
		.def("attachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link, bool attach) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, attach);
		}, py::arg("names"), py::arg("attach_link"), py::arg("attach") = true, R"pbdoc(
			attachObjects(attach_link, attach)

			Attach multiple objects to a robot link.

			Args:
				names (list): Objects that should be attached.
				attach_link (str): Link to which the objects should be attached.
				attach (bool): Set to true to attach the objects.
			Returns:
				None
		)pbdoc")
		.def("detachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, false);
		}, py::arg("names"), py::arg("attach_link"), R"pbdoc(
			detachObjects(attach_link)

			Detach multiple objects from a robot link.

			Args:
				names (list): Objects that should be attached.
				attach_link (str): Link from which the objects should be detached.
			Returns:
				None
		)pbdoc")
		.def("allowCollisions", [](ModifyPlanningScene& self,
	        const py::object& first, const py::object& second, bool enable_collision) {
			self.allowCollisions(elementOrList<std::string>(first), elementOrList<std::string>(second), enable_collision);
		}, py::arg("first"), py::arg("second"), py::arg("enable_collision") = true, R"pbdoc(
			allowCollisions(first, second, enable_collision)

			Allow or disable collisions between links and objects.

			Args:
				first (str): Name of the first object or link.
				second (str): Name of the second object or link.
				enable_collision (bool): Set to true to enable collisions checks;
										 set to false to disable collision checks.
			Returns:
				None
		)pbdoc")
		.def("addObject", &ModifyPlanningScene::addObject, R"pbdoc(
			addObject(collision_object)

			Add an object to the planning scene

			Args:
				collision_object (CollisionObject_): Object to be added. Must be
												     in the appropriate message format.
			Returns:
				None

			.. _CollisionObject: https://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/CollisionObject.html

		)pbdoc");

	properties::class_<CurrentState, Stage>(m, "CurrentState", R"pbdoc(
			CurrentState(name)

			Fetch the current PlanningScene state via the ``get_planning_scene`` service.

			Args:
				name (str): Name of the stage.

			.. literalinclude:: ./../../../demo/scripts/current_state.py
				:language: python

		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("current state"));

	properties::class_<FixedState, Stage>(m, "FixedState", R"pbdoc(
			FixedState(name)

			Spawn a pre-defined PlanningScene state.

			Args:
				name (str): Name of the stage.

			.. literalinclude:: ./../../../demo/scripts/fixed_state.py
				:language: python

		)pbdoc")
		.def("setState", &FixedState::setState, R"pbdoc(
			setState(scene)

			Use a planning scene pointer to specify which state the Fixed State
			stage should have.

			Args:
				scene (PlanningScenePtr): The desired planning scene state.
			Returns:
				None
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
			ComputeIK(name, stage)

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

			Args:
				name (str): Name of the stage.
				stage: Stage that contains the robot state for IK calculation.

			.. literalinclude:: ./../../../demo/scripts/compute_ik.py
				:language: python

		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: Specify which end effector of the active planning group
			should be used.
		)pbdoc")
	    .property<std::string>("group", R"pbdoc(
			str: Specify which planning group
			should be used.
		)pbdoc")
	    .property<std::string>("default_pose", R"pbdoc(
			str: Default joint pose of the active group
			(defines cost of the inverse kinematics).
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
			PoseStamped_: Specify the frame with respect
			to which the inverse kinematics
			should be calculated.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("target_pose", R"pbdoc(
			PoseStamped_: Specify the pose on which
			the inverse kinematics should be
			calculated on. Since this property should
			almost always be set
			in the Interface State which is sent by the child,
			if possible, avoid setting it manually.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    // methods of base class py::class_ need to be called last!
	    .def(py::init<const std::string&, Stage::pointer&&>());

	properties::class_<MoveTo, PropagatingEitherWay, PyMoveTo<>>(m, "MoveTo", R"pbdoc(
			MoveTo(name, planner)

			Compute a trajectory between the robot state from the
			interface state of the preceeding stage and a specified
			goal.

			Args:
				name (str): Name of the stage.
				planner (PlannerInterface): Planner that is used to compute the path of motion.

			.. literalinclude:: ./../../../demo/scripts/cartesian.py
				:language: python
				:lines: 51-55

		)pbdoc")
	    .property<std::string>("group", R"pbdoc(
			str: Planning group which should be utilized for planning and execution.
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("ik_frame", R"pbdoc(
			PoseStamped_: IK reference frame for the goal pose.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html

		)pbdoc")
	    .property<moveit_msgs::Constraints>("path_constraints", R"pbdoc(
			Constraints_: Set path constraints via the corresponding moveit message type.

			.. _Constraints: https://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
		)pbdoc")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	    .def("setGoal", py::overload_cast<const geometry_msgs::PoseStamped&>(&MoveTo::setGoal), R"pbdoc(
			setGoal(goal)

			1. Move link to a given pose.

			Args:
				goal (PoseStamped_): Desired configuration.
			Returns:
				None

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def("setGoal", py::overload_cast<const geometry_msgs::PointStamped&>(&MoveTo::setGoal), R"pbdoc(
			2. Move link to given point, keeping current orientation.

			Args:
				goal (PointStamped_): Desired configuration.
			Returns:
				None

			.. _PointStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PointStamped.html
		)pbdoc")
	    .def("setGoal", py::overload_cast<const moveit_msgs::RobotState&>(&MoveTo::setGoal), R"pbdoc(
			3. Move joints specified in msg to their target values.

			Args:
				goal (RobotState_): Desired configuration.
			Returns:
				None

			.. _RobotState: https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/RobotState.html
		)pbdoc")
	    .def("setGoal", py::overload_cast<const std::map<std::string, double>&>(&MoveTo::setGoal), R"pbdoc(
			4. Move joints by name to their mapped target value.

			Args:
				goal (dict): Desired configuration given in joint - value mappings.
			Returns:
				None
		)pbdoc")
	    .def("setGoal", py::overload_cast<const std::string&>(&MoveTo::setGoal), R"pbdoc(
			5. Move joint model group to given named pose.

			Args:
				goal (str): Desired configuration as a name of a known pose.
			Returns:
				None
		)pbdoc");

	properties::class_<MoveRelative, PropagatingEitherWay, PyMoveRelative<>>(m, "MoveRelative", R"pbdoc(
			MoveRelative(name, planner)

			Perform a Cartesian motion relative to some link.

			Args:
				name (str): Name of the stage.
				planner (PlannerInterface): Planner that is used to compute the path of motion.

			.. literalinclude:: ./../../../demo/scripts/cartesian.py
				:language: python
				:lines: 26-31

			To implement your own propagtor logic on top of the `moveRelative` class' functionality,
			you may derive from the stage like so:

			.. literalinclude:: ./../../python/test/rostest_trampoline.py
				:language: python
				:lines: 72-87


		)pbdoc")
	    .property<std::string>("group", R"pbdoc(
			str: Planning group which should be utilized for planning and execution.
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("ik_frame", R"pbdoc(
			PoseStamped_: IK reference frame for the goal pose.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .property<double>("min_distance", R"pbdoc(
			float: Set the minimum distance to move.
		)pbdoc")
	    .property<double>("max_distance", R"pbdoc(
			float: Set the maximum distance to move.
		)pbdoc")
	    .property<moveit_msgs::Constraints>("path_constraints", R"pbdoc(
			Constraints_: These are the path constraints.

			.. _Constraints: https://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
		)pbdoc")
	    .def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	    .def("setDirection", py::overload_cast<const geometry_msgs::TwistStamped&>(&MoveRelative::setDirection),
	        R"pbdoc(
			setDirection(twist)

			1. Perform twist motion on specified link.

			Args:
				twist (Twist_): Use a Twist message as movement direction description.
			Returns:
				None

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)pbdoc")
	    .def("setDirection", py::overload_cast<const geometry_msgs::Vector3Stamped&>(&MoveRelative::setDirection),
	        R"pbdoc(
			2. Translate link along given direction.

			Args:
				direction (Vector3Stamped_): Desired direction.
			Returns:
				None

			.. _Vector3Stamped: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3Stamped.html
		)pbdoc")
	    .def("setDirection", py::overload_cast<const std::map<std::string, double>&>(&MoveRelative::setDirection),
	        R"pbdoc(
			3. Move specified joint variables by given amount.

			Args:
				joint_deltas (dict): Desired direction,
					given as a (joint_name: str, joint_value: float), mapping.
			Returns:
				None
		)pbdoc");

	py::enum_<stages::Connect::MergeMode>(m, "MergeMode", R"pbdoc(
			Define the merge strategy to use when performing planning operations
			with e.g. the connect stage.
		)pbdoc")
	    .value("SEQUENTIAL", stages::Connect::MergeMode::SEQUENTIAL, R"pbdoc(
			Store sequential trajectories.
		)pbdoc")
	    .value("WAYPOINTS", stages::Connect::MergeMode::WAYPOINTS, R"pbdoc(
			Join trajectories by their waypoints.
		)pbdoc");
	PropertyConverter<stages::Connect::MergeMode>();

	properties::class_<Connect, Stage>(m, "Connect", R"pbdoc(
			Connect(name, planners)

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

 			Args:
				Name (str): Name of the stage.
				Planners (list): List of the planner - group associations.

			The example below contains a snippet from the :ref:`pick pipeline example<pick>`.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
			   :language: python
			   :lines: 48-60

		)pbdoc")
	    .def(py::init<const std::string&, const Connect::GroupPlannerVector&>(),
	         py::arg("name") = std::string("connect"), py::arg("planners"));

	properties::class_<FixCollisionObjects, Stage>(m, "FixCollisionObjects", R"pbdoc(
			FixCollisionObjects(name)

			Test for collisions and find a correction for applicable objects.
			Move the objects out of the way along the correction direction.

 			Args:
				name (str): Name of the stage.

			.. literalinclude:: ./../../../demo/scripts/fix_collision_objects.py
				:language: python

		)pbdoc")
	    .property<double>("max_penetration", R"pbdoc(
			float: Cutoff length up to which collision objects get fixed.
		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("fix collisions"));

	properties::class_<GeneratePlacePose, MonitoringGenerator>(m, "GeneratePlacePose", R"pbdoc(
			GeneratePlacePose(name)

			GeneratePlacePose stage derives from monitoring generator and generates poses
			for the place pipeline. Notice that whilst GenerateGraspPose spawns poses with an
			``angle_delta`` intervall, GeneratePlacePose samples a fixed amount, which is dependent
			on the objects shape.

			Args:
				name (str): Name of the stage.

			The example below contains a snippet from the :ref:`pick pipeline example<pick>`.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
				:language: python
				:lines: 115-122

		)pbdoc")
		.property<std::string>("object", R"pbdoc(
			str: Name of the object in the planning scene,
			attached to the robot which should be placed.
		)pbdoc")
		.property<std::string>("eef", R"pbdoc(
			str: Name of the end effector that should be used for grasping.
		)pbdoc")
		.property<geometry_msgs::PoseStamped>("pose")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("Generate Place Pose"));


	properties::class_<GenerateGraspPose, MonitoringGenerator>(m, "GenerateGraspPose", R"pbdoc(
			GenerateGraspPose(name)

			GenerateGraspPose stage derives from monitoring generator and can
			be used to generate poses for grasping. Set the desired attributes
			of the grasp using the stages properties.

			Args:
				name (str): Name of the stage.

			The example below contains a snippet from the :ref:`pick pipeline example<pick>`.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
			   :language: python
			   :lines: 62-68

		)pbdoc")
	    .property<std::string>("object", R"pbdoc(
			str: Name of the Object in the planning scene, which should be grasped.
		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: Name of the end effector that should be used for grasping.
		)pbdoc")
	    .property<std::string>("pregrasp", R"pbdoc(
			str: Name of the pre-grasp pose.
		)pbdoc")
	    .property<std::string>("grasp", R"pbdoc(
			str: Name of the grasp pose.
		)pbdoc")
	    .property<double>("angle_delta", R"pbdoc(
			float: Angular step distance in rad with which positions around the object are sampled.
		)pbdoc")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("Generate Grasp Pose"));

	properties::class_<GeneratePose, MonitoringGenerator>(m, "GeneratePose", R"pbdoc(
			GeneratePose(name)

			Monitoring generator stage which can be used to generate a pose, based on solutions provided
			by the monitored stage.

			Args:
				name (str): Name of the stage.

			.. literalinclude:: ./../../../demo/scripts/generate_pose.py
				:language: python
				:lines: 35-48
		)pbdoc")
	    .property<geometry_msgs::PoseStamped>("pose", R"pbdoc(
			PoseStamped_: Set the pose, which should be spawned on each new solution of the monitored stage.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def(py::init<const std::string&>());

	properties::class_<Pick, Stage>(m, "Pick", R"pbdoc(
			Pick(grasp_generator, name)

			Args:
				grasp_generator (SimpleGrasp): Simple Grasp stage to provide
											   poses and inverse kinematics solutions.
				name (str): Name of the stage.

			.. _pick:

			The Pick stage is a specialization of the PickPlaceBase class, which
			wraps the pipeline to pick or place an object with a given end effector.

			Picking consist of the following sub stages:

				- Linearly approaching the object along an approach direction/twist "grasp" end effector posture
				- Attach the object
				- Lift along a given direction/twist

			The end effector postures corresponding to pre-grasp and grasp as well
			as the end effector's cartesian pose needs to be provided by an external
			grasp stage.

			The example below combines pick and place routines in a single pickplace pipeline.
			Please refer to the demonstration scripts of the moveit task constructor repository
			for reference.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
				:language: python

		)pbdoc")
	    .property<std::string>("object", R"pbdoc(
			str: Name of object to pick.
		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: The End effector name.
		)pbdoc")
	    .property<std::string>("eef_frame", R"pbdoc(
			str: Name of the end effector frame.
		)pbdoc")
	    .property<std::string>("eef_group", R"pbdoc(
			str: Joint model group of the end effector.
		)pbdoc")
	    .property<std::string>("eef_parent_group", R"pbdoc(
			str: Joint model group of the eef's parent.
		)pbdoc")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("grasp_generator"),
	         py::arg("name") = std::string("pick"))
	    .def("setApproachMotion", &Pick::setApproachMotion, R"pbdoc(
			setApproachMotion(motion, min_distance, max_distance)

			Args:
				motion (Twist_): The twist, which represents the
								 approach motion.
				min_distance (float): Minimum allowed distance.
				max_distance (float): Maximum allowed distance.
			Returns:
				None

			The approaching motion towards the grasping state is represented
			by a twist message.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)pbdoc")
	    .def("setLiftMotion", py::overload_cast<const geometry_msgs::TwistStamped&, double, double>(&Pick::setLiftMotion), R"pbdoc(
			setLiftMotion(motion, min_distance, max_distance)

			1. The lifting motion away from the grasping state is represented by a twist message.

			Args:
				motion (Twist_): The twist, which represents the
								 lift motion.
				min_distance (float): Minimum allowed distance.
				max_distance (float): Maximum allowed distance.
			Returns:
				None

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)pbdoc")
	    .def("setLiftMotion", py::overload_cast<const std::map<std::string, double>&>(&Pick::setLiftMotion), R"pbdoc(
			2. The lifting motion away from the grasping state is represented by its destination as joint-value pairs.

			Args:
				place (dict): The place where the object should be lifted to,
							  given as joint-value pairs.
			Returns:
				None
		)pbdoc");

	properties::class_<Place, Stage>(m, "Place", R"pbdoc(
			Place(place_generator, name)

			Args:
				place_generator (SimpleUnGrasp): SimpleUnGrasp Wrapper for pose generation
				name (str): Name of the stage.

			The Place stage is a specialization of the PickPlaceBase class, which
			wraps the pipeline to pick or place an object with a given end effector.

			Placing consist of the inverse order of stages:

				- Place down along a given direction
				- Detach the object
				- Linearly retract end effector

			The end effector postures corresponding to pre-grasp and grasp as well
			as the end effector's Cartesian pose needs to be provided by an external
			grasp stage.

			For a working example, please consider the Pick_ Stage.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
				:language: python
				:lines: 131-135
		)pbdoc")
	    .property<std::string>("object", R"pbdoc(
			str: Name of object to pick.
		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: The End effector name.
		)pbdoc")
	    .property<std::string>("eef_frame", R"pbdoc(
			str: Name of the end effector frame.
		)pbdoc")
	    .property<std::string>("eef_group", R"pbdoc(
			str: Joint model group of the end effector.
		)pbdoc")
	    .property<std::string>("eef_parent_group", R"pbdoc(
			str: Joint model group of the eef's parent.
		)pbdoc")
    	.def("setRetractMotion", &Place::setRetractMotion, R"pbdoc(
			setRetractMotion(motion, min_distance, max_distance)

			Args:
				motion (Twist_): The twist, which represents the
								 retract motion.
				min_distance (float): Minimum allowed distance.
				max_distance (float): Maximum allowed distance.
			Returns:
				None

			The retract motion towards the final state is represented
			by a twist message.

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
		)pbdoc")
	    .def("setPlaceMotion", py::overload_cast<const geometry_msgs::TwistStamped&, double, double>(&Place::setPlaceMotion), R"pbdoc(
			setPlaceMotion(motion, min_distance, max_distance)

			1. The object-placing motion towards the final state is represented by a twist message.

			Args:
				motion (Twist_): The twist, which represents the
								 place motion.
				min_distance (float): Minimum allowed distance.
				max_distance (float): Maximum allowed distance.
			Returns:
				None

			.. _Twist: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html

		)pbdoc")
	    .def("setPlaceMotion", py::overload_cast<const std::map<std::string, double>&>(&Place::setPlaceMotion), R"pbdoc(
			2. The placing motion to the final state is represented by its destination as joint-value pairs.

			Args:
				joints (dict): The place where the object should be placed at,
							   given as joint-value pairs.
			Returns:
				None
		)pbdoc")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("place_generator"),
	         py::arg("name") = std::string("place"));

	properties::class_<SimpleGrasp, Stage>(m, "SimpleGrasp", R"pbdoc(
			SimpleGrasp(pose_generator, name)

			Args:
				pose_generator (GenerateGraspPose): Generator stage to
													sample possible grasp poses.
				name (str): Name of the stage.
			Returns:
				None

			Specialization of SimpleGraspBase to realize grasping.
			Refer to the pick_ stage for a minimum code example:

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
				:language: python
				:lines: 70-79
		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: The end effector of the robot.
		)pbdoc")
	    .property<std::string>("object", R"pbdoc(
			str: The object to grasp (Must be present in the planning scene).
		)pbdoc")
		.property<geometry_msgs::PoseStamped>("ik_frame", R"pbdoc(
			PoseStamped_: Set the frame for which
			the inverse kinematics are calculated
			with respect to each pose generated by
			the pose_generator.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("pose_generator"),
	         py::arg("name") = std::string("grasp generator"))
	    .def<void (SimpleGrasp::*)(const geometry_msgs::PoseStamped&)>("setIKFrame", &SimpleGrasp::setIKFrame, R"pbdoc(
			setIKFrame(transform)

			1. Set the frame for which the inverse kinematics are calculated with respect to
			   each pose generated by the pose_generator.

			Args:
				transform (PoseStamped_): Transform to the IK Frame.
			Returns:
				None

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def<void (SimpleGrasp::*)(const Eigen::Isometry3d&, const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame, R"pbdoc(
			2. Set the frame for which the inverse kinematics are calculated
			   with respect to each pose generated by the pose_generator.

			Args:
				pose (PoseStamped_): Transform from the given link to the IK frame.
				link (str): Base link for pose transform to IK frame.
			Returns:
				None
		)pbdoc")
	    .def<void (SimpleGrasp::*)(const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame, R"pbdoc(
			3. Set the frame for which the inverse kinematics are calculated
			   with respect to each pose generated by the pose_generator.

			Args:
				link (str): IK Frame will be placed at the base frame of this link.
			Returns:
				None
		)pbdoc")
		.def("setMaxIKSolutions", &SimpleGrasp::setMaxIKSolutions, R"pbdoc(
			setMaxIKSolutions(max_ik_solutions)

			Args:
				max_ik_solutions (int): Maximum number of ik solutions.
			Returns:
				None

			Set the maximum number of inverse kinematics solutions that
			should be computed.

		)pbdoc");

	properties::class_<SimpleUnGrasp, Stage>(m, "SimpleUnGrasp", R"pbdoc(
			SimpleUnGrasp(pose_generator, name)

			Args:
				pose_generator (GeneratePlacePose): Generator stage to
													sample possible Place
													poses.
				name (str): Name of the stage.
			Returns:
				None

			Specialization of SimpleGraspBase to realize ungrasping
			Refer to the place_ stage for a minimum code example.
			Make shure to set the ``grasp`` and ``pregrasp`` properties
			here again since they are not forwarded through the stage
			hierarchy.

			.. literalinclude:: ./../../../demo/scripts/pickplace.py
				:language: python
				:lines: 124-129

		)pbdoc")
	    .property<std::string>("eef", R"pbdoc(
			str: The end effector of the robot.
		)pbdoc")
	    .property<std::string>("object", R"pbdoc(
			str: The object to grasp (Must be present in the planning scene).
		)pbdoc")
		.property<std::string>("pregrasp", R"pbdoc(
			str: Name of the pre-grasp pose.
		)pbdoc")
		.property<std::string>("grasp", R"pbdoc(
			str: Name of the grasp pose.
		)pbdoc")
		.property<geometry_msgs::PoseStamped>("ik_frame", R"pbdoc(
			PoseStamped_: Specify the frame with respect
			to which the inverse kinematics
			should be calculated.

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def<void (SimpleUnGrasp::*)(const geometry_msgs::PoseStamped&)>("setIKFrame", &SimpleUnGrasp::setIKFrame, R"pbdoc(
			setIKFrame(transform)

			1. Set the frame for which the inverse kinematics are calculated
			   with respect to each pose generated by the pose_generator.

			Args:
				transform (PoseStamped_): Transform to the IK Frame.
			Returns:
				None

			.. _PoseStamped: https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
		)pbdoc")
	    .def<void (SimpleUnGrasp::*)(const Eigen::Isometry3d&, const std::string&)>("setIKFrame", &SimpleUnGrasp::setIKFrame, R"pbdoc(
			2. Set the frame for which the inverse kinematics are calculated
			   with respect to each pose generated by the pose_generator.

			Args:
				pose (PoseStamped_): Transform from the given link to the IK frame.
									 link (str): Base link for pose transform to IK frame.
			Returns:
				None
		)pbdoc")
	    .def<void (SimpleUnGrasp::*)(const std::string&)>("setIKFrame", &SimpleUnGrasp::setIKFrame, R"pbdoc(
			3. Set the frame for which the inverse kinematics are calculated
			   with respect to each pose generated by the pose_generator.

			Args:
				link (str): IK Frame will be placed at the base frame of this link.
			Returns:
				None
		)pbdoc")
		.def("setMaxIKSolutions", &SimpleUnGrasp::setMaxIKSolutions, R"pbdoc(
			setMaxIKSolutions(max_ik_solutions)

			Args:
				max_ik_solutions (int): Maximum number of ik solutions.
			Returns:
				None

			Set the maximum number of inverse kinematics solutions that
			should be computed.

		)pbdoc")
	    .def(py::init<Stage::pointer&&, const std::string&>(), py::arg("pose_generator"),
	         py::arg("name") = std::string("place generator"));
}
}  // namespace python
}  // namespace moveit
