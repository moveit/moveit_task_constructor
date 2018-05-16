#include <moveit/python/python_tools/conversions.h>
#include <ros/serialization.h>

namespace moveit {
namespace python {

struct MessageSignature {
	/// constructor from name of form package/type
	MessageSignature(const boost::python::type_info& type_info);
	/// constructor from name of form package/type
	MessageSignature(const std::string& message);
	/// constructor from python object's class name
	explicit MessageSignature(PyObject* object);

	bool isValid() const {
		return !module.empty() && !message.empty();
	}

	std::string module; // "<package name>.msg"
	std::string message;
};
inline bool operator<(const MessageSignature& __x, const MessageSignature& __y)
{
	return __x.module < __y.module
	      || (!(__y.module < __x.module) && __x.message < __y.message);
}

/// constructor from name of form package/message
MessageSignature::MessageSignature(const boost::python::type_info& type_info)
{
	const std::string type_name(type_info.name());
	size_t pos = type_name.find("::");
	if (pos != std::string::npos) {
		this->module = type_name.substr(0, pos) + ".msg";

		size_t end = type_name.find("_<", pos+2);
		if (end != std::string::npos) {
			this->message = type_name.substr(pos+2, end - (pos+2));
			return;
		}
	}
	throw std::invalid_argument("Invalid C++ type name for ROS msg: " + type_name);
}

MessageSignature::MessageSignature(const std::string& type_name) {
	size_t pos = type_name.find('/');
	if (pos == std::string::npos)
		throw std::invalid_argument("Invalid ROS msg name: " + type_name);
	this->module = type_name.substr(0, pos) + ".msg";
	this->message = type_name.substr(pos+1);
}

/// constructor from object's module name of form package.msg._type
MessageSignature::MessageSignature(PyObject* object) {
	static const std::string key = ".msg._";

	boost::python::object o(boost::python::borrowed(object));
	std::string module = boost::python::extract<std::string>(o.attr("__class__").attr("__module__"));

	size_t pos = module.find(key, 0);
	if (pos == std::string::npos)
		return; // not a ROS msg object

	this->module = module.substr(0, pos + 4);
	this->message = module.substr(pos + key.size());
}


/// Additionally to the boost::python::converter::registry, this class remembers the registered
/// ROS messages and their signature (in the form of package/msg).
class ConverterRegistry {
	typedef std::map<MessageSignature, boost::python::object> SignatureMap;
	SignatureMap signatures_;
	typedef std::map<boost::python::type_info, SignatureMap::iterator> TypeIdMap;
	TypeIdMap types_;

public:
	bool insert(const MessageSignature &s, const boost::python::type_info& type_info) {
		auto it_inserted = signatures_.insert(std::make_pair(s, boost::python::object()));
		if (!it_inserted.second)
			return false; // was already inserted before
		auto type_it_inserted = types_.insert(std::make_pair(type_info, it_inserted.first));
		// type should have been registered too with same name
		assert(type_it_inserted.second && type_it_inserted.first->second == it_inserted.first);
		return true;
	}

	/// check whether object can be converted to C++ type
	inline void* convertible(PyObject* object);

	/// instantiate a new python object for given ROS msg type
	boost::python::object createMessage(const boost::python::type_info& type_info);
};
static ConverterRegistry registry_singleton_;

inline void* ConverterRegistry::convertible(PyObject* object)
{
	MessageSignature signature(object);
	if (signature.isValid() && signatures_.find(signature) != signatures_.end())
		return object;
	else
		return 0;
}

boost::python::object ConverterRegistry::createMessage(const boost::python::type_info& type_info)
{
	auto type_it = types_.find(type_info);
	if (type_it == types_.end())
		throw std::runtime_error(std::string("No converter registered for: ") + type_info.name());

	auto signature_it = type_it->second;
	boost::python::object& cls = signature_it->second;

	if (cls.is_none()) {  // load msg module once on demand
		const MessageSignature& s = signature_it->first;
		std::string module_name = s.module;
		boost::python::object module = boost::python::import(module_name.c_str());
		cls = module.attr(s.message.c_str());
	}
	return cls();
}


bool ROSMsgConverterBase::insert(const boost::python::type_info& type_info, const std::string& message)
{
	MessageSignature signature = message.empty() ? MessageSignature(type_info) : MessageSignature(message);
	return registry_singleton_.insert(signature, type_info);
}

void* ROSMsgConverterBase::convertible(PyObject* object) {
	/// object is convertible if the type of registered
	return registry_singleton_.convertible(object);
}

std::string fromPython(const boost::python::object& msg)
{
	boost::python::object StringIO = boost::python::import("StringIO");
	boost::python::object buf = StringIO.attr("StringIO")();
	msg.attr("serialize")(buf);
	return boost::python::extract<std::string>(buf.attr("getvalue")());
}

PyObject* toPython(const std::string& data, const boost::python::type_info& type_info)
{
	boost::python::object msg = registry_singleton_.createMessage(type_info);
	msg.attr("deserialize")(boost::python::object(data));
	return boost::python::incref(msg.ptr());
}

} }
