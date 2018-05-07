#include <boost/python.hpp>

namespace moveit {
namespace python {

void export_ros_init();
void export_task();

} }

BOOST_PYTHON_MODULE(_core)
{
	moveit::python::export_ros_init();
	moveit::python::export_task();
}
