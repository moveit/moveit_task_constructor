#include <boost/python/to_python_converter.hpp>
#include <boost/python/object.hpp>
#include <boost/python/handle.hpp>
#include <boost/python/extract.hpp>
#include <ros/duration.h>

namespace moveit {
namespace python {

// convert python Time/Duration/double into ros::Duration
struct Duration_from_python
{
	static const char* getter;

	// Determine if obj can be converted into ros::Duration
	static void* convertible(PyObject* obj)
	{
		// either expect a double or an object providing a "to_sec" function
		if (!PyFloat_Check(obj) && !PyObject_HasAttrString(obj, getter))
			return 0;
		else
			return obj;
	}

	// Convert obj into a ros::Duration
	static void construct(PyObject* obj,
	                      boost::python::converter::rvalue_from_python_stage1_data* data)
	{
		double value = 0.0;
		if (PyObject_HasAttrString(obj, getter)) {
			boost::python::handle<> handle(boost::python::borrowed(obj));
			boost::python::object getter_function = boost::python::object(handle).attr(getter);
			value = boost::python::extract<double>(getter_function());
		} else if (PyFloat_Check(obj))
			value = PyFloat_AS_DOUBLE(obj);
		else
			assert(false);  // should not happen

		// Obtain a pointer to the memory block that the converter has allocated for the C++ type.
		void* storage = reinterpret_cast<boost::python::converter::rvalue_from_python_storage<ros::Duration>*>(data)->storage.bytes;
		// Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's convertible variable.
		data->convertible = new (storage) ros::Duration(value);
	}
};
const char* Duration_from_python::getter = "to_sec";

struct Duration_to_double {
	static PyObject* convert(const ros::Duration& x) {
		return PyFloat_FromDouble(x.toSec());
	}
};
struct WallDuration_to_double {
	static PyObject* convert(const ros::WallDuration& x) {
		return PyFloat_FromDouble(x.toSec());
	}
};


class ConverterInit {
public:
	ConverterInit() {
		using namespace boost::python;
		converter::registry::push_back(&Duration_from_python::convertible, &Duration_from_python::construct,
		                               boost::python::type_id<ros::Duration>());
		to_python_converter<ros::Duration, Duration_to_double>();
		to_python_converter<ros::WallDuration, WallDuration_to_double>();
	}
};
static ConverterInit init;

} }
