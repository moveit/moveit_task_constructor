#include <boost/python.hpp>

#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::solvers;

namespace moveit {
namespace python {

namespace {


} // anonymous namespace

void export_solvers()
{
	PropertyMap& (PlannerInterface::*PlannerInterface_getPropertyMap)() = &PlannerInterface::properties;
	properties::class_<PlannerInterface, PlannerInterfacePtr, boost::noncopyable>("PlannerInterface", bp::no_init)
	      .property<double>("max_velocity_scaling_factor")
	      .property<double>("max_acceleration_scaling_factor")
	      .add_property("properties", bp::make_function(PlannerInterface_getPropertyMap, bp::return_internal_reference<>()))
	      ;

	properties::class_<PipelinePlanner, PipelinePlannerPtr, bp::bases<PlannerInterface>>
	      ("PipelinePlanner", bp::init<>())
	      .property<std::string>("group")
	      .property<std::string>("planner")
	      .property<double>("timeout")
	      ;
	bp::implicitly_convertible<PipelinePlannerPtr, PlannerInterfacePtr>();

	properties::class_<CartesianPath, CartesianPathPtr, bp::bases<PlannerInterface>>
	      ("CartesianPath", bp::init<>())
	      .property<std::string>("group")
	      .property<double>("timeout")
	      .property<double>("step_size")
	      .property<double>("jump_threshold")
	      .property<double>("min_fraction")
	      ;
	bp::implicitly_convertible<CartesianPathPtr, PlannerInterfacePtr>();
}

} }
