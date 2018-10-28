#include <moveit/python/python_tools/conversions.h>
#include <ros/serialization.h>

namespace moveit {
namespace python {

std::string rosMsgName(PyObject* object) {
	boost::python::object o(boost::python::borrowed(object));
	try {
		return boost::python::extract<std::string>(o.attr("_type"));
	} catch (const boost::python::error_already_set&) {
		// change error to TypeError
		std::string msg = boost::python::extract<std::string>(o.attr("__class__").attr("__name__"));
		msg += " is not a ROS message type";
		PyErr_SetString(PyExc_TypeError, msg.c_str());
		throw;
	}
}

namespace {
/// Additionally to the boost::python::converter::registry, this class remembers the registered
/// ROS messages and their signature (in the form of package/msg).
class RosMsgConverterRegistry {
	// map from ros-msg-names to corresponding Python type instances
	typedef std::map<std::string, boost::python::object> RegistryMap;
	RegistryMap signatures_;
	// map from typeinfo to entry in signatures_
	typedef std::map<boost::python::type_info, RegistryMap::iterator> TypeIdMap;
	TypeIdMap types_;

public:
	/// register type_info as convertible between C++ and Python
	inline bool insert(const boost::python::type_info& type_info, const std::string& ros_msg_name);

	/// check whether object can be converted to C++ type
	inline void* convertible(PyObject* object);

	/// instantiate a new python object for given ROS msg type
	inline boost::python::object createMessage(const boost::python::type_info& type_info);
};
static RosMsgConverterRegistry registry_singleton_;

bool RosMsgConverterRegistry::insert(const boost::python::type_info& type_info, const std::string& ros_msg_name) {
	auto it_inserted = signatures_.insert(std::make_pair(ros_msg_name, boost::python::object()));
	if (!it_inserted.second)
		return false; // was already inserted before
	auto type_it_inserted = types_.insert(std::make_pair(type_info, it_inserted.first));
	// type should have been registered too with same name
	assert(type_it_inserted.second && type_it_inserted.first->second == it_inserted.first);
	(void)type_it_inserted;
	return true;
}

inline void* RosMsgConverterRegistry::convertible(PyObject* object)
{
	try {
		const std::string& ros_msg_name = rosMsgName(object);
		if (signatures_.find(ros_msg_name) != signatures_.end())
			return object;
	} catch (const std::invalid_argument&) {}
	return 0;
}

boost::python::object RosMsgConverterRegistry::createMessage(const boost::python::type_info& type_info)
{
	auto type_it = types_.find(type_info);
	if (type_it == types_.end())
		throw std::runtime_error(std::string("No converter registered for: ") + type_info.name());

	auto signature_it = type_it->second;
	boost::python::object& cls = signature_it->second;

	if (cls.is_none()) {  // load msg module once on demand
		const std::string& ros_msg_name = signature_it->first;
		// find delimiting '/' in ros msg name
		std::size_t pos = ros_msg_name.find('/');
		// import module
		boost::python::object m = boost::python::import((ros_msg_name.substr(0, pos) + ".msg").c_str());
		// retrieve type instance (and remember it in signature_it->second)
		cls = m.attr(ros_msg_name.substr(pos+1).c_str());
	}
	return cls();
}

} // anonymous namespace

bool RosMsgConverterBase::insert(const boost::python::type_info& type_info, const std::string& ros_msg_name)
{
	return registry_singleton_.insert(type_info, ros_msg_name);
}

void* RosMsgConverterBase::convertible(PyObject* object) {
	/// object is convertible if the type of registered
	return registry_singleton_.convertible(object);
}

std::string RosMsgConverterBase::fromPython(const boost::python::object& msg)
{
	boost::python::object StringIO = boost::python::import("StringIO");
	boost::python::object buf = StringIO.attr("StringIO")();
	msg.attr("serialize")(buf);
	return boost::python::extract<std::string>(buf.attr("getvalue")());
}

PyObject* RosMsgConverterBase::toPython(const std::string& data, const boost::python::type_info& type_info)
{
	boost::python::object msg = registry_singleton_.createMessage(type_info);
	msg.attr("deserialize")(boost::python::object(data));
	return boost::python::incref(msg.ptr());
}

} }
