#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

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
	bp::class_<CurrentState, bp::bases<Stage>, boost::noncopyable>
	      ("CurrentState", bp::init<bp::optional<const std::string&>>())
	      ;

	bp::class_<FixedState, bp::bases<Stage>, boost::noncopyable>
	      ("FixedState", bp::init<bp::optional<const std::string&>>())
	      .def("setState", &FixedState::setState)
	      ;

	bp::class_<Connect, bp::bases<Stage>, boost::noncopyable>
	      ("Connect", bp::no_init)
	      .def("__init__", bp::make_constructor(&initConnect))
	      ;
}

} }
