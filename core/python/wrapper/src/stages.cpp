#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <moveit/python/python_tools/conversions.h>
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


// TODO optional arguments for constructors? relevant for Compute_IK, Pick, Place and Task (in core.cpp)
ComputeIK* ComputeIK_init(const std::string& name, std::auto_ptr<Stage> stage) {
	return new ComputeIK(name, std::unique_ptr<Stage>{stage.release()});
}

bp::list ComputeIK_getForwardedProperties(ComputeIK& self) {
	std::set<std::string> forward_properties = self.properties().get<std::set<std::string>>("forward_properties");
	bp::list l;
	for (const std::string& value : forward_properties)
		l.append(value);
	return l;
}

void ComputeIK_setForwardedProperties(ComputeIK& self, const bp::list& names) {
	boost::python::stl_input_iterator<std::string> begin(names), end;
	self.setForwardedProperties(std::set<std::string>(begin, end));
}


bp::dict MoveRelative_getJoints(MoveRelative& self) {
	return toDict<double>(self.properties().get<std::map<std::string, double>>("joints"));
}

void MoveRelative_setJoints(MoveRelative& self, const bp::dict& joints) {
	self.about(fromDict<double>(joints));
}


Connect* Connect_init(const std::string& name, const bp::list& l) {
	Connect::GroupPlannerVector planners;
	for (bp::stl_input_iterator<bp::tuple> it(l), end; it != end; ++it) {
		const std::string& group = bp::extract<std::string>((*it)[0]);
		solvers::PlannerInterfacePtr solver = bp::extract<solvers::PlannerInterfacePtr>((*it)[1]);
		planners.push_back(std::make_pair(group, solver));
	}
	return new Connect(name, planners);
}


Pick* Pick_init(std::auto_ptr<Stage> grasp_stage, const std::string& name) {
	return new Pick(std::unique_ptr<Stage>{grasp_stage.release()}, name);
}


Place* Place_init(std::auto_ptr<Stage> ungrasp_stage, const std::string& name) {
	return new Place(std::unique_ptr<Stage>{ungrasp_stage.release()}, name);
}

} // anonymous namespace

void export_stages()
{
	// register type converters
	ROSMsgConverter<geometry_msgs::PoseStamped>();
	ROSMsgConverter<geometry_msgs::Pose>();
	ROSMsgConverter<geometry_msgs::TwistStamped>();
	ROSMsgConverter<geometry_msgs::Vector3Stamped>();
	ROSMsgConverter<moveit_msgs::RobotState>();
	ROSMsgConverter<moveit_msgs::Constraints>();


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
	      .property<double>("timeout")
	      .property<std::string>("eef")
	      .property<std::string>("group")
	      .property<std::string>("default_pose")
	      .property<uint32_t>("max_ik_solutions")
	      .property<bool>("ignore_collisions")
	      .property<geometry_msgs::PoseStamped>("ik_frame")
	      .property<geometry_msgs::PoseStamped>("target_pose")
	      // methods of base class boost::python::class_ need to be called last!
	      .def("__init__", bp::make_constructor(&ComputeIK_init))
	      .add_property("forward_properties", &ComputeIK_getForwardedProperties, &ComputeIK_setForwardedProperties) // TODO test
	      ;
	bp::implicitly_convertible<std::auto_ptr<ComputeIK>, std::auto_ptr<Stage>>();


	properties::class_<MoveTo, std::auto_ptr<MoveTo>, bp::bases<Stage>, boost::noncopyable>
	      ("MoveTo", bp::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	      .property<double>("timeout")
	      .property<std::string>("group")
	      .property<std::string>("link")
	      .property<geometry_msgs::PoseStamped>("pose")
	      .property<geometry_msgs::PointStamped>("point")
	      .property<std::string>("named_joint_pose")
	      .property<moveit_msgs::RobotState>("joint_pose")
	      .property<moveit_msgs::Constraints>("path_constraints")
	      ;
	bp::implicitly_convertible<std::auto_ptr<MoveTo>, std::auto_ptr<Stage>>();


	properties::class_<MoveRelative, std::auto_ptr<MoveRelative>, bp::bases<Stage>, boost::noncopyable>
	      ("MoveRelative", bp::init<const std::string&, const solvers::PlannerInterfacePtr&>())
	      .property<double>("timeout")
	      .property<std::string>("marker_ns")
	      .property<std::string>("group")
	      .property<std::string>("link")
	      .property<double>("min_distance")
	      .property<double>("max_distance")
	      .property<geometry_msgs::TwistStamped>("twist")
	      .property<geometry_msgs::Vector3Stamped>("direction")
	      .property<moveit_msgs::Constraints>("path_constraints")
	      .add_property("joints", &MoveRelative_getJoints, &MoveRelative_setJoints)
	      ;
	bp::implicitly_convertible<std::auto_ptr<MoveRelative>, std::auto_ptr<Stage>>();


	properties::class_<Connect, std::auto_ptr<Connect>, bp::bases<Stage>, boost::noncopyable>
	      ("Connect", bp::no_init)
	      // use a custom wrapper as constructor to pass a python list of (name, planner) tuples
	      .def("__init__", bp::make_constructor(&Connect_init))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Connect>, std::auto_ptr<Stage>>();


	properties::class_<FixCollisionObjects, std::auto_ptr<FixCollisionObjects>, bp::bases<Stage>, boost::noncopyable>
	      ("FixCollisionObjects", bp::init<bp::optional<const std::string&>>())
	      .property<double>("max_penetration")
	      ;
	bp::implicitly_convertible<std::auto_ptr<FixCollisionObjects>, std::auto_ptr<Stage>>();


	properties::class_<GenerateGraspPose, std::auto_ptr<GenerateGraspPose>, bp::bases<Stage>, boost::noncopyable>
	      ("GenerateGraspPose", bp::init<const std::string&>())
	      .property<std::string>("eef")
	      .property<std::string>("pregrasp")
	      .property<std::string>("object")
	      .property<double>("angle_delta")
	      ;
	bp::implicitly_convertible<std::auto_ptr<GenerateGraspPose>, std::auto_ptr<Stage>>();


	properties::class_<GeneratePose, std::auto_ptr<GeneratePose>, bp::bases<Stage>, boost::noncopyable>
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

	      .def("__init__", bp::make_constructor(&Pick_init))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Pick>, std::auto_ptr<Stage>>();


	properties::class_<Place, std::auto_ptr<Place>, bp::bases<Stage>, boost::noncopyable>
	      ("Place", bp::no_init)
	      .property<std::string>("object")
	      .property<std::string>("eef")
	      .property<std::string>("eef_frame")
	      .property<std::string>("eef_group")
	      .property<std::string>("eef_parent_group")

	      .def("__init__", bp::make_constructor(&Place_init))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Place>, std::auto_ptr<Stage>>();


	properties::class_<SimpleGrasp, std::auto_ptr<SimpleGrasp>, bp::bases<Stage>, boost::noncopyable>
	      ("SimpleGrasp", bp::init<bp::optional<const std::string&>>())
	      .property<std::string>("eef")
	      .property<std::string>("object")
	      ;
	bp::implicitly_convertible<std::auto_ptr<SimpleGrasp>, std::auto_ptr<Stage>>();


	properties::class_<SimpleUnGrasp, std::auto_ptr<SimpleUnGrasp>, bp::bases<Stage>, boost::noncopyable>
	      ("SimpleUnGrasp", bp::init<bp::optional<const std::string&>>())
	      .property<std::string>("eef")
	      .property<std::string>("object")
	      ;
	bp::implicitly_convertible<std::auto_ptr<SimpleUnGrasp>, std::auto_ptr<Stage>>();
}

} }
