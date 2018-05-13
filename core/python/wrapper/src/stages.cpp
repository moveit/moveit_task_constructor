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

#include <moveit/task_constructor/stages/pick.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

namespace moveit {
namespace python {

namespace {

Connect* initConnect(const std::string& name, const bp::list& l) {
	Connect::GroupPlannerVector planners;
	for (bp::stl_input_iterator<bp::tuple> it(l), end; it != end; ++it) {
		const std::string& group = bp::extract<std::string>((*it)[0]);
		solvers::PlannerInterfacePtr solver = bp::extract<solvers::PlannerInterfacePtr>((*it)[1]);
		planners.push_back(std::make_pair(group, solver));
	}
	return new Connect(name, planners);
}

} // anonymous namespace

void export_stages()
{
	properties::class_<CurrentState, std::auto_ptr<CurrentState>, bp::bases<Stage>, boost::noncopyable>
	      ("CurrentState", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<CurrentState>, std::auto_ptr<Stage>>();

	properties::class_<FixedState, std::auto_ptr<FixedState>, bp::bases<Stage>, boost::noncopyable>
	      ("FixedState", bp::init<bp::optional<const std::string&>>())
	      .def("setState", &FixedState::setState)
	      ;
	bp::implicitly_convertible<std::auto_ptr<FixedState>, std::auto_ptr<Stage>>();

	properties::class_<Connect, std::auto_ptr<Connect>, bp::bases<Stage>, boost::noncopyable>
	      ("Connect", bp::no_init)
	      // use a custom wrapper as constructor to pass a python list of (name, planner) tuples
	      .def("__init__", bp::make_constructor(&initConnect))
	      ;
	bp::implicitly_convertible<std::auto_ptr<Connect>, std::auto_ptr<Stage>>();
}

} }
