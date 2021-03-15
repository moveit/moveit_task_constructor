#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit_msgs/PlanningScene.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

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
	properties::class_<ModifyPlanningScene, Stage>(m, "ModifyPlanningScene")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("modify planning scene"))
		.def("attachObject", &ModifyPlanningScene::attachObject)
		.def("detachObject", &ModifyPlanningScene::detachObject)
		.def("attachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link, bool attach) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, attach);
		}, py::arg("names"), py::arg("attach_link"), py::arg("attach") = true)
		.def("detachObjects", [](ModifyPlanningScene& self, const py::object& names,
		                         const std::string& attach_link) {
			self.attachObjects(elementOrList<std::string>(names), attach_link, false);
		}, py::arg("names"), py::arg("attach_link"))
		.def("allowCollisions", [](ModifyPlanningScene& self,
	        const py::object& first, const py::object& second, bool enable_collision) {
			self.allowCollisions(elementOrList<std::string>(first), elementOrList<std::string>(second), enable_collision);
		}, py::arg("first"), py::arg("second"), py::arg("enable_collision") = true);

	properties::class_<CurrentState, Stage>(m, "CurrentState")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("current state"));

	properties::class_<FixedState, Stage>(m, "FixedState")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("fixed state"))
      #if 0
		.def("setState", [](FixedState& stage, const moveit_msg::PlanningScene& scene_msg) {
			// TODO: How to initialize the PlanningScene?
			planning_scene::PlanningScenePtr scene;
			scene->setPlanningSceneMsg(scene_msg);
			stage.setState(scene);
		})
#endif
		;

	properties::class_<ComputeIK, Stage>(m, "ComputeIK")
		.property<std::string>("eef")
		.property<std::string>("group")
		.property<std::string>("default_pose")
		.property<uint32_t>("max_ik_solutions")
		.property<bool>("ignore_collisions")
		.property<geometry_msgs::PoseStamped>("ik_frame")
		.property<geometry_msgs::PoseStamped>("target_pose")
		// methods of base class boost::python::class_ need to be called last!
		.def(py::init<const std::string&, Stage::pointer&&>());

	properties::class_<MoveTo, PropagatingEitherWay>(m, "MoveTo")
		.property<std::string>("group")
		.property<geometry_msgs::PoseStamped>("ik_frame")
		.property<moveit_msgs::Constraints>("path_constraints")
		.def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
		.def<void (MoveTo::*)(const geometry_msgs::PoseStamped&)>("setGoal", &MoveTo::setGoal)
		.def<void (MoveTo::*)(const geometry_msgs::PointStamped&)>("setGoal", &MoveTo::setGoal)
		.def<void (MoveTo::*)(const moveit_msgs::RobotState&)>("setGoal", &MoveTo::setGoal)
		.def<void (MoveTo::*)(const std::map<std::string, double>&)>("setGoal", &MoveTo::setGoal)
		.def<void (MoveTo::*)(const std::string&)>("setGoal", &MoveTo::setGoal);

	properties::class_<MoveRelative, PropagatingEitherWay>(m, "MoveRelative")
		.property<std::string>("group")
		.property<geometry_msgs::PoseStamped>("ik_frame")
		.property<double>("min_distance")
		.property<double>("max_distance")
		.property<moveit_msgs::Constraints>("path_constraints")
		.def(py::init<const std::string&, const solvers::PlannerInterfacePtr&>())
		.def<void (MoveRelative::*)(const geometry_msgs::TwistStamped&)>("setDirection", &MoveRelative::setDirection)
		.def<void (MoveRelative::*)(const geometry_msgs::Vector3Stamped&)>("setDirection", &MoveRelative::setDirection)
		.def<void (MoveRelative::*)(const std::map<std::string, double>&)>("setDirection", &MoveRelative::setDirection);

	py::enum_<stages::Connect::MergeMode>(m, "MergeMode")
		.value("SEQUENTIAL", stages::Connect::MergeMode::SEQUENTIAL)
		.value("WAYPOINTS", stages::Connect::MergeMode::WAYPOINTS);
	PropertyConverter<stages::Connect::MergeMode>();

	properties::class_<Connect, Stage>(m, "Connect")
		.def(py::init<const std::string&, const Connect::GroupPlannerVector&>(),
			py::arg("name") = std::string("connect"), py::arg("planners"));

	properties::class_<FixCollisionObjects, Stage>(m, "FixCollisionObjects")
		.property<double>("max_penetration")
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

	properties::class_<Pick, Stage>(m, "Pick")
		.property<std::string>("object")
		.property<std::string>("eef")
		.property<std::string>("eef_frame")
		.property<std::string>("eef_group")
		.property<std::string>("eef_parent_group")
		.def(py::init<Stage::pointer&&, const std::string&>(),
		     py::arg("grasp_generator"), py::arg("name") = std::string("pick"))

		.def("setApproachMotion", &Pick::setApproachMotion)
		.def<void (Pick::*)(const geometry_msgs::TwistStamped&, double, double)>("setLiftMotion", &Pick::setLiftMotion)
		.def<void (Pick::*)(const std::map<std::string, double>&)>("setLiftMotion", &Pick::setLiftMotion)
		;

	properties::class_<Place, Stage>(m, "Place")
		.property<std::string>("object")
		.property<std::string>("eef")
		.property<std::string>("eef_frame")
		.property<std::string>("eef_group")
		.property<std::string>("eef_parent_group")
		.def(py::init<Stage::pointer&&, const std::string&>(),
	        py::arg("place_generator"), py::arg("name") = std::string("place"));

	properties::class_<SimpleGrasp, Stage>(m, "SimpleGrasp")
		.property<std::string>("eef")
		.property<std::string>("object")
		.def(py::init<Stage::pointer&&, const std::string&>(),
	        py::arg("pose_generator"), py::arg("name") = std::string("grasp generator"))
		.def<void (SimpleGrasp::*)(const geometry_msgs::PoseStamped&)>("setIKFrame", &SimpleGrasp::setIKFrame)
		.def<void (SimpleGrasp::*)(const Eigen::Isometry3d&, const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame)
		.def<void (SimpleGrasp::*)(const std::string&)>("setIKFrame", &SimpleGrasp::setIKFrame)
		;

	properties::class_<SimpleUnGrasp, Stage>(m, "SimpleUnGrasp")
		.property<std::string>("eef")
		.property<std::string>("object")
		.def(py::init<Stage::pointer&&, const std::string&>(),
	        py::arg("pose_generator"), py::arg("name") = std::string("place generator"));
}
}  // namespace python
}  // namespace moveit
