#include <boost/python/to_python_converter.hpp>
#include <boost/python/object.hpp>
#include <boost/python/handle.hpp>
#include <boost/python/extract.hpp>
#include <ros/duration.h>

namespace bp = boost::python;

namespace moveit {
namespace python {

// Converter for python Time/Duration/double into ros::Duration / ros::WallDuration
template <typename T>
struct DurationConverter
{
	// Determine if obj can be converted into duration
	static void* convertible(PyObject* obj)
	{
		bp::object bpo(bp::borrowed(obj));
		// either expect a double or an object providing a "to_sec" function
		if (PyFloat_Check(obj) ||
		    (PyObject_HasAttrString(obj, "to_sec") && PyFloat_Check(bpo.attr("to_sec")().ptr())))
			return obj;
		else
			return 0;
	}

	// Convert obj into a duration
	static void construct(PyObject* obj,
	                      bp::converter::rvalue_from_python_stage1_data* data)
	{
		double value = 0.0;
		if (PyObject_HasAttrString(obj, "to_sec")) {
			bp::object bpo(bp::borrowed(obj));
			value = bp::extract<double>(bpo.attr("to_sec")());
		} else if (PyFloat_Check(obj)) {
			value = PyFloat_AS_DOUBLE(obj);
		} else {  // should not happen
			PyErr_SetString(PyExc_TypeError, "unexpected type");
			bp::throw_error_already_set();
		}

		// Obtain a pointer to the memory block that the converter has allocated for the C++ type.
		void* storage = reinterpret_cast<bp::converter::rvalue_from_python_storage<T>*>(data)->storage.bytes;
		// Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's convertible variable.
		data->convertible = new (storage) T(value);
	}

	// Convert duration to python
	static PyObject* convert(const T& x) {
		return PyFloat_FromDouble(x.toSec());
	}

	DurationConverter() {  // constructor registers type converter with boost::python
		bp::converter::registry::push_back(&DurationConverter<T>::convertible, &DurationConverter::construct,
		                                   bp::type_id<T>());
		bp::to_python_converter<T, DurationConverter<T>>();
	}
};

class ConverterInit {
public:
	ConverterInit() {
		DurationConverter<ros::Duration>();
		DurationConverter<ros::WallDuration>();
	}
};
static ConverterInit init;

} }
