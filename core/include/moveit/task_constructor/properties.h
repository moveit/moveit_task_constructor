/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   PropertyMap stores variables of stages
*/

#pragma once

#include <boost/any.hpp>
#include <typeindex>
#include <map>
#include <set>
#include <vector>
#include <functional>
#include <sstream>

namespace moveit {
namespace task_constructor {

class Property;
class PropertyMap;

/// initializer function, using given name from the passed property map
boost::any fromName(const PropertyMap& other, const std::string& other_name);

/** Property is a wrapper for a boost::any value, also providing a default value and a description.
 *
 * Properties can be configured to be initialized from another PropertyMap - if still undefined.
 * A source id allows to distinguish different initialization methods (e.g. using a different reference
 * name) as well as to define a priority order between sources.
 *
 * Setting the value via setValue() updates both, the current value and the default value.
 * Using reset() the default value can be restored.
 * Using setCurrentValue() only updates the current value, allowing for later reset to the original default.
 */
class Property {
	friend class PropertyMap;

	typedef std::function<std::string(const boost::any& v)> SerializeFunction;

	Property(const std::type_index &type_index, const std::string &description, const boost::any &default_value,
	         const Property::SerializeFunction &serialize);

	template <typename T>
	static std::string serialize(const boost::any& value) {
		if (value.empty()) return "";
		std::ostringstream oss;
		oss << boost::any_cast<T>(value);
		return oss.str();
	}

public:
	/// base class for Property exceptions
	class error;
	/// exception thrown when accessing an undeclared property
	class undeclared;
	/// exception thrown when accessing an undefined property
	class undefined;
	/// exception thrown when trying to set a value not matching the declared type
	class type_error;

	typedef int SourceId;
	/// function callback used to initialize property value from another PropertyMap
	typedef std::function<boost::any(const PropertyMap& other)> InitializerFunction;
	/// function callback used to signal value setting to external components
	typedef std::function<void(const Property*)> SignalFunction;

	/// set current value and default value
	void setValue(const boost::any& value);
	void setCurrentValue(const boost::any& value);

	/// reset to default value (which can be empty)
	void reset();

	/// the current value defined or will the fallback be used?
	inline bool defined() const { return !value_.empty(); }

	/// get current value (or default if not defined)
	inline const boost::any& value() const { return value_.empty() ? default_ : value_; }
	/// get default value
	const boost::any& defaultValue() const { return default_; }
	/// serialize current value
	std::string serialize() const;

	/// get description text
	const std::string& description() const { return description_; }
	/// get typename
	std::string typeName() const { return type_index_.name(); }

	/// return true, if property initialized from given SourceId
	bool initsFrom(SourceId source) const;
	/// configure initialization from source using an arbitrary function
	Property &configureInitFrom(SourceId source, const InitializerFunction& f);
	/// configure initialization from source using given other property name
	Property &configureInitFrom(SourceId source, const std::string& name);

	/// set current value using matching configured initializers
	void performInitFrom(SourceId source, const PropertyMap& other);

	/// define a function callback to be called on each value update
	/// note, that boost::any doesn't allow for change detection
	void setSignalCallback(const SignalFunction& f) { signaller_ = f; }

private:
	std::string description_;
	std::type_index type_index_;
	boost::any default_;
	boost::any value_;

	/// used for external initialization
	SourceId source_id_ = 0;
	InitializerFunction initializer_;
	SignalFunction signaller_;
	SerializeFunction serialize_;
};


class Property::error : public std::runtime_error {
protected:
	std::string property_name_;
	std::string msg_;
public:
	explicit error(const std::string& msg);
	const std::string& name() const { return property_name_; }
	void setName(const std::string& name);
	const char* what() const noexcept override { return msg_.c_str(); }
};
class Property::undeclared : public Property::error {
public:
	explicit undeclared(const std::string& name, const std::string& msg = "undeclared");
};
class Property::undefined : public Property::error {
public:
	explicit undefined(const std::string& name, const std::string& msg = "undefined");
};
class Property::type_error : public Property::error {
public:
	explicit type_error(const std::string& current_type, const std::string& declared_type);
};


/** PropertyMap is map of (name, Property) pairs.
 *
 * Conveniency methods are provided to setup property initialization for several
 * properties at once - always inheriting from the identically named external property.
 */
class PropertyMap
{
	std::map<std::string, Property> props_;
	typedef std::map<std::string, Property>::iterator iterator;
	typedef std::map<std::string, Property>::const_iterator const_iterator;

	/// implementation of declare methods
	Property& declare(const std::string& name, const std::type_index& type_index,
	                  const std::string& description,
	                  const boost::any& default_value,
	                  const Property::SerializeFunction &serialize);
public:
	/// declare a property for future use
	template<typename T>
	Property& declare(const std::string& name, const std::string& description = "") {
		return declare(name, typeid(T), description, boost::any(), &Property::serialize<T>);
	}
	/// declare a property with default value
	template<typename T>
	Property& declare(const std::string& name, const T& default_value,
	             const std::string& description = "") {
		return declare(name, typeid(T), description, default_value, &Property::serialize<T>);
	}
	/// declare all given properties also in other PropertyMap
	void exposeTo(PropertyMap& other, const std::set<std::string>& properties);

	/// declare given property name as other_name in other PropertyMap
	void exposeTo(PropertyMap& other, const std::string& name, const std::string& other_name);

	bool hasProperty(const std::string &name) const;

	/// get the property with given name, throws Property::undeclared for unknown name
	Property& property(const std::string &name);
	const Property& property(const std::string &name) const {
		return const_cast<PropertyMap*>(this)->property(name);
	}

	iterator begin() { return props_.begin(); }
	iterator end() { return props_.end(); }
	const_iterator begin() const { return props_.begin(); }
	const_iterator end() const { return props_.end(); }

	/// allow initialization from given source for listed properties - always using the same name
	void configureInitFrom(Property::SourceId source, const std::set<std::string> &properties = {});

	/// set (and, if neccessary, declare) the value of a property
	template <typename T>
	void set(const std::string& name, const T& value) {
		auto it = props_.find(name);
		if (it == props_.end())  // name is not yet declared
			declare<T>(name, value, "");
		else
			it->second.setValue(value);
	}

	/// overloading: const char* is stored as std::string
	inline void set(const std::string& name, const char* value){
		set<std::string>(name, value);
	}

	/// temporarily set the value of a property
	void setCurrent(const std::string& name, const boost::any& value);

	/// Gt the value of a property. Throws undeclared if unknown name
	const boost::any& get(const std::string& name) const;

	/// Get typed value of property. Throws undeclared, undefined, or bad_any_cast.
	template<typename T>
	const T& get(const std::string& name) const {
		const boost::any& value = get(name);
		if (value.empty())
			throw Property::undefined(name);
		return boost::any_cast<const T&>(value);
	}
	/// get typed value of property, using fallback if undefined. Throws bad_any_cast on type mismatch.
	template<typename T>
	const T& get(const std::string& name, const T& fallback) const {
		const boost::any& value = get(name);
		return (value.empty()) ? fallback : boost::any_cast<const T&>(value);
	}

	/// count number of defined properties from given list
	size_t countDefined(const std::vector<std::string>& list) const;

	/// reset all properties to their defaults
	void reset();

	/// perform initialization of still undefined properties using configured initializers
	void performInitFrom(Property::SourceId source, const PropertyMap& other, bool enforce = false);
};

// boost::any needs a specialization to avoid recursion
template <>
void PropertyMap::set<boost::any>(const std::string& name, const boost::any& value);

// provide a serialization method for std::map
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::map<std::string, T>& m) {
    os << "{";
    bool first = true;
    for (const auto& pair : m) {
        if (!first)
            os << ", ";
        os << pair.first << " : " << pair.second;
        first = false;
    }
    os << "}";
}

} // namespace task_constructor
} // namespace moveit
