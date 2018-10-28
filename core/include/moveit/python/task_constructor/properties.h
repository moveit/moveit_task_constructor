#include <boost/python/class.hpp>
#include <boost/mpl/vector.hpp>

#include <moveit/task_constructor/properties.h>
#include <moveit/python/python_tools/conversions.h>

namespace moveit {
namespace python {

void export_properties();

class PropertyConverterBase {
public:
	typedef boost::python::object (*to_python_converter_function)(const boost::any&);
	typedef boost::any (*from_python_converter_function)(const boost::python::object&);

protected:
	static bool insert(const boost::python::type_info& type_info, const std::string& ros_msg_name,
	                   to_python_converter_function to,
	                   from_python_converter_function from);
};

/// utility class to register C++ / Python converters for a property of type T
template <typename T>
class PropertyConverter : protected PropertyConverterBase {
public:
	PropertyConverter() {
		auto type_info = boost::python::type_id<T>();
		insert(type_info, rosMsgName<T>(), &toPython, &fromPython);
	}

private:
	static boost::python::object toPython(const boost::any& value) {
		return boost::python::object(boost::any_cast<T>(value));
	}
	static boost::any fromPython(const boost::python::object& bpo) {
		return T(boost::python::extract<T>(bpo));
	}

	template <class Q = T>
	typename std::enable_if<ros::message_traits::IsMessage<Q>::value, std::string>::type
	rosMsgName() {
		RosMsgConverter<T>();  // register ROS msg converter
		return ros::message_traits::DataType<T>::value();
	}

	template <class Q = T>
	typename std::enable_if<!ros::message_traits::IsMessage<Q>::value, std::string>::type
	rosMsgName() { return std::string(); }  // empty string if T isn't a ROS msg
};

namespace properties {

/** Extension for boost::python::class_ to allow convienient definition of properties
 *
 * New method property<PropertyType>(const char* name) adds a property getter/setter.
 */

template <
    class W // class being wrapped
    , class X1 = ::boost::python::detail::not_specified
    , class X2 = ::boost::python::detail::not_specified
    , class X3 = ::boost::python::detail::not_specified
    >
class class_ : public boost::python::class_<W, X1, X2, X3>
{
public:
	typedef class_<W,X1,X2,X3> self;
	// forward all constructors
	using boost::python::class_<W, X1, X2, X3>::class_;

	template <typename PropertyType>
	self& property(const char* name, const char* docstr = 0) {
		auto getter = [name](const W& me) {
			const moveit::task_constructor::PropertyMap& props = me.properties();
			return props.get<PropertyType>(name);
		};
		auto setter = [name](W& me, const PropertyType& value) {
			moveit::task_constructor::PropertyMap& props = me.properties();
			props.set(name, boost::any(value));
		};

		boost::python::class_<W, X1, X2, X3>::add_property
		      (name, boost::python::make_function
		       (getter, boost::python::default_call_policies(),
		        boost::mpl::vector<PropertyType, const W&>()),
		       boost::python::make_function
		       (setter, boost::python::default_call_policies(),
		        boost::mpl::vector<void, W&, const PropertyType&>()),
		       docstr);
		return *this;
	}
};

} } }
