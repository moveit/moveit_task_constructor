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
	bp::class_<PlannerInterface, PlannerInterfacePtr, boost::noncopyable>("PlannerInterface", bp::no_init)
	      .add_property("properties", bp::make_function(PlannerInterface_getPropertyMap, bp::return_internal_reference<>()))
	      ;

	properties::class_<PipelinePlanner, PipelinePlannerPtr, bp::bases<PlannerInterface>>
	      ("PipelinePlanner", bp::init<>())
	      .add_property<std::string>("group")
	      .add_property<std::string>("planner")
	      .add_property<double>("timeout")
	      ;

	properties::class_<CartesianPath, CartesianPathPtr, bp::bases<PlannerInterface>>
	      ("CartesianPath", bp::init<>())
	      .add_property<std::string>("group")
	      .add_property<double>("timeout")
	      .add_property<double>("step_size")
	      .add_property<double>("jump_threshold")
	      .add_property<double>("min_fraction")
	      .add_property<double>("max_velocity_scaling_factor")
	      .add_property<double>("max_acceleration_scaling_factor")
	      ;
}

} }
