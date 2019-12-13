#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_msgs/WorkspaceParameters.h>

namespace py = pybind11;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::solvers;

namespace moveit {
namespace python {

void export_solvers(py::module& m) {
	// resolve &PlannerInterface::properties to non-const version
	PropertyMap& (PlannerInterface::*PlannerInterface_getPropertyMap)() = &PlannerInterface::properties;

	// clang-format off
	properties::class_<PlannerInterface, PlannerInterfacePtr>(m, "PlannerInterface")
		.property<double>("max_velocity_scaling_factor")
		.property<double>("max_acceleration_scaling_factor")
		.def_property_readonly("properties", PlannerInterface_getPropertyMap, py::return_value_policy::reference_internal)
		;

	properties::class_<PipelinePlanner, PipelinePlannerPtr, PlannerInterface>(m, "PipelinePlanner")
		.property<std::string>("planner")
		.property<uint>("num_planning_attempts")
		.property<moveit_msgs::WorkspaceParameters>("workspace_parameters")
		.property<double>("goal_joint_tolerance")
		.property<double>("goal_position_tolerance")
		.property<double>("goal_orientation_tolerance")
		.property<bool>("display_motion_plans")
		.property<bool>("publish_planning_requests")
		.def(py::init<>())
		;

	properties::class_<JointInterpolationPlanner, JointInterpolationPlannerPtr, PlannerInterface>(m, "JointInterpolationPlanner")
		.property<double>("max_step")
		.def(py::init<>())
		;

	properties::class_<CartesianPath, CartesianPathPtr, PlannerInterface>(m, "CartesianPath")
		.property<double>("step_size")
		.property<double>("jump_threshold")
		.property<double>("min_fraction")
		.def(py::init<>())
		;
	// clang-format on
}
}  // namespace python
}  // namespace moveit
