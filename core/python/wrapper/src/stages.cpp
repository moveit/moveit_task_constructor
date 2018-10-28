#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

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

namespace bp = boost::python;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

namespace moveit {
namespace python {

namespace {

/// extract from python argument a vector<T>, where arg maybe a single T or a list of Ts
template <typename T>
std::vector<T> elementOrList(const bp::object& arg) {
	bp::extract<T> arg_as_T(arg);
	if (arg_as_T.check())
		return std::vector<T>{arg_as_T()};
	else
		return fromList<std::string>(bp::extract<bp::list>(arg));
}


void ModifyPlanningScene_attachObjects(ModifyPlanningScene& self, const bp::object& names,
                                       const std::string& attach_link, bool attach = true) {
	self.attachObjects(elementOrList<std::string>(names), attach_link, attach);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ModifyPlanningScene_attachObjects_overloads, ModifyPlanningScene_attachObjects, 3, 4)

void ModifyPlanningScene_detachObjects(ModifyPlanningScene& self, const bp::object& names,
                                       const std::string& attach_link) {
	ModifyPlanningScene_attachObjects(self, names, attach_link, false);
}

void ModifyPlanningScene_allowCollisions(ModifyPlanningScene& self, const bp::object& first,
                                         const bp::object& second, bool enable_collision = true) {
	self.allowCollisions(elementOrList<std::string>(first), elementOrList<std::string>(second), enable_collision);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ModifyPlanningScene_allowCollisions_overloads, ModifyPlanningScene_allowCollisions, 3, 4)


std::auto_ptr<ComputeIK> ComputeIK_init_2(const std::string& name, std::auto_ptr<Stage> stage) {
	return std::auto_ptr<ComputeIK>(new ComputeIK(name, std::unique_ptr<Stage>{stage.release()}));
}
std::auto_ptr<ComputeIK> ComputeIK_init_1(const std::string& name) {
	return std::auto_ptr<ComputeIK>(new ComputeIK(name));
}
std::auto_ptr<ComputeIK> ComputeIK_init_0() {
	return std::auto_ptr<ComputeIK>(new ComputeIK());
}


void MoveRelative_setJoints(MoveRelative& self, const bp::dict& joints) {
	self.setDirection(fromDict<double>(joints));
}


std::auto_ptr<Connect> Connect_init_2(const std::string& name, const bp::list& l) {
	Connect::GroupPlannerVector planners;
	for (bp::stl_input_iterator<bp::tuple> it(l), end; it != end; ++it) {
		std::string group = bp::extract<std::string>((*it)[0]);
		solvers::PlannerInterfacePtr solver = bp::extract<solvers::PlannerInterfacePtr>((*it)[1]);
		planners.push_back(std::make_pair(group, solver));
	}
	return std::auto_ptr<Connect>(new Connect(name, planners));
}
std::auto_ptr<Connect> Connect_init_1(const std::string& name) {
	return Connect_init_2(name, bp::list());
}
std::auto_ptr<Connect> Connect_init_0() {
	return Connect_init_2(std::string(), bp::list());
}

std::auto_ptr<Pick> Pick_init_2(std::auto_ptr<Stage> grasp_stage, const std::string& name) {
	return std::auto_ptr<Pick>(new Pick(std::unique_ptr<Stage>{grasp_stage.release()}, name));
}
std::auto_ptr<Pick> Pick_init_1(std::auto_ptr<Stage> grasp_stage) {
	return Pick_init_2(std::move(grasp_stage), std::string());
}
std::auto_ptr<Pick> Pick_init_0() {
	return Pick_init_2(std::auto_ptr<Stage>(), std::string());
}


std::auto_ptr<Place> Place_init_2(std::auto_ptr<Stage> ungrasp_stage, const std::string& name) {
	return std::auto_ptr<Place>(new Place(std::unique_ptr<Stage>{ungrasp_stage.release()}, name));
}
std::auto_ptr<Place> Place_init_1(std::auto_ptr<Stage> ungrasp_stage) {
	return Place_init_2(std::move(ungrasp_stage), std::string());
}
std::auto_ptr<Place> Place_init_0() {
	return Place_init_2(std::auto_ptr<Stage>(), std::string());
}


std::auto_ptr<SimpleGrasp> SimpleGrasp_init_2(std::auto_ptr<Stage> pose_gen, const std::string& name) {
	return std::auto_ptr<SimpleGrasp>(new SimpleGrasp(std::unique_ptr<Stage>{pose_gen.release()}, name));
}
std::auto_ptr<SimpleGrasp> SimpleGrasp_init_1(std::auto_ptr<Stage> pose_gen) {
	return SimpleGrasp_init_2(std::move(pose_gen), std::string());
}
std::auto_ptr<SimpleGrasp> SimpleGrasp_init_0() {
	return SimpleGrasp_init_2(std::auto_ptr<Stage>(), std::string());
}


std::auto_ptr<SimpleUnGrasp> SimpleUnGrasp_init_2(std::auto_ptr<Stage> pose_gen, const std::string& name) {
	return std::auto_ptr<SimpleUnGrasp>(new SimpleUnGrasp(std::unique_ptr<Stage>{pose_gen.release()}, name));
}
std::auto_ptr<SimpleUnGrasp> SimpleUnGrasp_init_1(std::auto_ptr<Stage> pose_gen) {
	return SimpleUnGrasp_init_2(std::move(pose_gen), std::string());
}
std::auto_ptr<SimpleUnGrasp> SimpleUnGrasp_init_0() {
	return SimpleUnGrasp_init_2(std::auto_ptr<Stage>(), std::string());
}

} // anonymous namespace

void export_stages()
{
	// register type converters for properties
	PropertyConverter<geometry_msgs::PoseStamped>();
	PropertyConverter<geometry_msgs::Pose>();
	PropertyConverter<geometry_msgs::TwistStamped>();
	PropertyConverter<geometry_msgs::Vector3Stamped>();
	PropertyConverter<geometry_msgs::PointStamped>();
	PropertyConverter<moveit_msgs::RobotState>();
	PropertyConverter<moveit_msgs::Constraints>();


	properties::class_<ModifyPlanningScene, std::auto_ptr<ModifyPlanningScene>, bp::bases<Stage>, boost::noncopyable>
	      ("ModifyPlanningScene", bp::init<bp::optional<const std::string&>>())
	      .def("attachObject", &ModifyPlanningScene::attachObject)
	      .def("detachObject", &ModifyPlanningScene::detachObject)
	      .def("attachObjects", &ModifyPlanningScene_attachObjects, ModifyPlanningScene_attachObjects_overloads())
	      .def("detachObjects", &ModifyPlanningScene_detachObjects)
	      .def("allowCollisions", &ModifyPlanningScene_allowCollisions, ModifyPlanningScene_allowCollisions_overloads())
	      ;
	bp::implicitly_convertible<std::auto_ptr<ModifyPlanningScene>, std::auto_ptr<Stage>>();


	properties::class_<CurrentState, std::auto_ptr<CurrentState>, bp::bases<Stage>, boost::noncopyable>
	      ("CurrentState", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<CurrentState>, std::auto_ptr<Stage>>();


	properties::class_<FixedState, std::auto_ptr<FixedState>, bp::bases<Stage>, boost::noncopyable>
	      ("FixedState", bp::init<bp::optional<const std::string&>>())
	      .def("setState", &FixedState::setState)
	      ;
	bp::implicitly_convertible<std::auto_ptr<FixedState>, std::auto_ptr<Stage>>();


	properties::class_<ComputeIK, std::auto_ptr<ComputeIK>, bp::bases<Stage>, boost::noncopyable>
	      ("ComputeIK", bp::no_init)
	      .property<std::string>("eef")
	      .property<std::string>("group")
	      .property<std::string>("default_pose")
	      .property<uint32_t>("max_ik_solutions")
	      .property<bool>("ignore_collisions")
	      .property<geometry_msgs::PoseStamped>("ik_frame")
	      .property<geometry_msgs::PoseStamped>("target_pose")
	      // methods of base class boost::python::class_ need to be called last!
	      .def("__init__", bp::make_constructor(&ComputeIK_init_2))
	      .def("__init__", bp::make_constructor(&ComputeIK_init_1))
	      .def("__init__", bp::make_constructor(&ComputeIK_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<ComputeIK>, std::auto_ptr<Stage>>();


	void (MoveTo::*setGoalPose)(const geometry_msgs::PoseStamped&) = &MoveTo::setGoal;
	void (MoveTo::*setGoalPoint)(const geometry_msgs::PointStamped&) = &MoveTo::setGoal;
	void (MoveTo::*setGoalState)(const moveit_msgs::RobotState&) = &MoveTo::setGoal;
	void (MoveTo::*setGoalNamed)(const std::string&) = &MoveTo::setGoal;
	properties::class_<MoveTo, std::auto_ptr<MoveTo>, bp::bases<Stage>, boost::noncopyable>
	      ("MoveTo", bp::init<bp::optional<const std::string&, const solvers::PlannerInterfacePtr&>>())
	      .property<std::string>("group")
	      .property<geometry_msgs::PoseStamped>("ik_frame")
	      .property<moveit_msgs::Constraints>("path_constraints")
	      .def("setGoal", setGoalPose)
	      .def("setGoal", setGoalPoint)
	      .def("setGoal", setGoalState)
	      .def("setGoal", setGoalNamed)
	      ;
	bp::implicitly_convertible<std::auto_ptr<MoveTo>, std::auto_ptr<Stage>>();


	void (MoveRelative::*setTwist)(const geometry_msgs::TwistStamped&) = &MoveRelative::setDirection;
	void (MoveRelative::*setDirection)(const geometry_msgs::Vector3Stamped&) = &MoveRelative::setDirection;
	properties::class_<MoveRelative, std::auto_ptr<MoveRelative>, bp::bases<Stage>, boost::noncopyable>
	      ("MoveRelative", bp::init<bp::optional<const std::string&, const solvers::PlannerInterfacePtr&>>())
	      .property<std::string>("group")
	      .property<geometry_msgs::PoseStamped>("ik_frame")
	      .property<double>("min_distance")
	      .property<double>("max_distance")
	      .property<moveit_msgs::Constraints>("path_constraints")
	      .def("setDirection", setTwist)
	      .def("setDirection", setDirection)
	      .def("setDirection", &MoveRelative_setJoints)
	      ;
	bp::implicitly_convertible<std::auto_ptr<MoveRelative>, std::auto_ptr<Stage>>();


	bp::enum_<stages::Connect::MergeMode>("MergeMode")
	      .value("SEQUENTIAL", stages::Connect::MergeMode::SEQUENTIAL)
	      .value("WAYPOINTS", stages::Connect::MergeMode::WAYPOINTS)
	      ;
	PropertyConverter<stages::Connect::MergeMode>();

	properties::class_<Connect, std::auto_ptr<Connect>, bp::bases<Stage>, boost::noncopyable>
	      ("Connect", bp::no_init)
	      // use a custom wrapper as constructor to pass a python list of (name, planner) tuples
	      .def("__init__", bp::make_constructor(&Connect_init_2))
	      .def("__init__", bp::make_constructor(&Connect_init_1))
	      .def("__init__", bp::make_constructor(&Connect_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Connect>, std::auto_ptr<Stage>>();


	properties::class_<FixCollisionObjects, std::auto_ptr<FixCollisionObjects>, bp::bases<Stage>, boost::noncopyable>
	      ("FixCollisionObjects", bp::init<bp::optional<const std::string&>>())
	      .property<double>("max_penetration")
	      ;
	bp::implicitly_convertible<std::auto_ptr<FixCollisionObjects>, std::auto_ptr<Stage>>();


	properties::class_<GenerateGraspPose, std::auto_ptr<GenerateGraspPose>, bp::bases<MonitoringGenerator>, boost::noncopyable>
	      ("GenerateGraspPose", bp::init<const std::string&>())
	      .property<std::string>("object")
	      .property<std::string>("eef")
	      .property<std::string>("pregrasp")
	      .property<std::string>("grasp")
	      .property<double>("angle_delta")
	      ;
	bp::implicitly_convertible<std::auto_ptr<GenerateGraspPose>, std::auto_ptr<Stage>>();


	properties::class_<GeneratePose, std::auto_ptr<GeneratePose>, bp::bases<MonitoringGenerator>, boost::noncopyable>
	      ("GeneratePose", bp::init<const std::string&>())
	      .property<geometry_msgs::PoseStamped>("pose")
	      ;
	bp::implicitly_convertible<std::auto_ptr<GeneratePose>, std::auto_ptr<Stage>>();


	properties::class_<Pick, std::auto_ptr<Pick>, bp::bases<Stage>, boost::noncopyable>
	      ("Pick", bp::no_init)
	      .property<std::string>("object")
	      .property<std::string>("eef")
	      .property<std::string>("eef_frame")
	      .property<std::string>("eef_group")
	      .property<std::string>("eef_parent_group")

	      .def("__init__", bp::make_constructor(&Pick_init_2))
	      .def("__init__", bp::make_constructor(&Pick_init_1))
	      // .def("__init__", bp::make_constructor(&Pick_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Pick>, std::auto_ptr<Stage>>();


	properties::class_<Place, std::auto_ptr<Place>, bp::bases<Stage>, boost::noncopyable>
	      ("Place", bp::no_init)
	      .property<std::string>("object")
	      .property<std::string>("eef")
	      .property<std::string>("eef_frame")
	      .property<std::string>("eef_group")
	      .property<std::string>("eef_parent_group")

	      .def("__init__", bp::make_constructor(&Place_init_2))
	      .def("__init__", bp::make_constructor(&Place_init_1))
	      // .def("__init__", bp::make_constructor(&Place_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Place>, std::auto_ptr<Stage>>();


	properties::class_<SimpleGrasp, std::auto_ptr<SimpleGrasp>, bp::bases<Stage>, boost::noncopyable>
	      ("SimpleGrasp", bp::no_init)
	      .property<std::string>("eef")
	      .property<std::string>("object")
	      .def("__init__", bp::make_constructor(&SimpleGrasp_init_2))
	      .def("__init__", bp::make_constructor(&SimpleGrasp_init_1))
	      // .def("__init__", bp::make_constructor(&SimpleGrasp_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<SimpleGrasp>, std::auto_ptr<Stage>>();


	properties::class_<SimpleUnGrasp, std::auto_ptr<SimpleUnGrasp>, bp::bases<Stage>, boost::noncopyable>
	      ("SimpleUnGrasp", bp::no_init)
	      .property<std::string>("eef")
	      .property<std::string>("object")
	      .def("__init__", bp::make_constructor(&SimpleUnGrasp_init_2))
	      .def("__init__", bp::make_constructor(&SimpleUnGrasp_init_1))
	      // .def("__init__", bp::make_constructor(&SimpleUnGrasp_init_0))
	      ;
	bp::implicitly_convertible<std::auto_ptr<SimpleUnGrasp>, std::auto_ptr<Stage>>();
}

} }
