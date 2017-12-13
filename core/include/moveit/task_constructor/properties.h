#pragma once

#include <boost/any.hpp>
#include <typeindex>
#include <map>
#include <set>
#include <functional>

namespace moveit {
namespace task_constructor {

class Property;
class PropertyMap;

/// initializer function, using another name from the passed property map
boost::any fromName(const PropertyMap& other, const std::string& other_name);

enum PropertyInitializerSource {
	PARENT,
	INTERFACE,
};

class Property {
	friend class PropertyMap;

public:
	typedef std::function<boost::any(const PropertyMap& other, const std::string& other_name)> InitializerFunction;
	typedef std::map<PropertyInitializerSource, InitializerFunction> InitializerMap;

	Property(const std::type_index& type_index, const std::string& description, const boost::any& default_value);

	void setValue(const boost::any& value);

	const boost::any& value() const;
	const boost::any& defaultValue() const { return default_; }
	const std::string& description() const { return description_; }
	std::string typeName() const { return type_index_.name(); }

	/// set an initializer function
	Property &initFrom(PropertyInitializerSource source, const InitializerFunction& f = fromName);
	Property &initFrom(PropertyInitializerSource source, const std::string& other_name);

private:
	std::string description_;
	std::type_index type_index_;
	boost::any default_;
	boost::any value_;
	InitializerMap initializers_;
};


class PropertyMap
{
	std::map<std::string, Property> props_;

public:
	/// declare a property for future use
	Property& declare(const std::string& name, const std::type_info& type,
	                  const std::string& description = "",
	                  const boost::any& default_value = boost::any());

	template<typename T>
	Property& declare(const std::string& name, const std::string& description = "") {
		return declare(name, typeid(T), description);
	}
	template<typename T>
	Property& declare(const std::string& name, const T& default_value,
	             const std::string& description = "") {
		return declare(name, typeid(T), description, default_value);
	}

	/// get the property with given name
	Property& property(const std::string &name);
	const Property& property(const std::string &name) const {
		return const_cast<PropertyMap*>(this)->property(name);
	}

	/// allow initialization from given source for listed properties
	void initFrom(PropertyInitializerSource source, const std::set<std::string> &properties = {});

	/// set the value of a property
	void set(const std::string& name, const boost::any& value);

	/// get the value of a property
	const boost::any& get(const std::string& name) const;
	template<typename T>
	const T& get(const std::string& name) const {
		return boost::any_cast<const T&>(get(name));
	}

	/// reset properties to nil, if they have initializers
	void reset();
	/// init properties from initializers
	void initFrom(PropertyInitializerSource source, const PropertyMap& other,
	              bool checkConsistency = false);
};

} // namespace task_constructor
} // namespace moveit
