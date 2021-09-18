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

#include <moveit_task_constructor_msgs/Property.h>

#include "properties/serialize.hpp"

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
 * setCurrentValue() and setDefaultValue() only set the specific value.
 */
class Property
{
	friend class PropertyMap;

	/// typed constructor is only accessible via PropertyMap
	Property(const boost::typeindex::type_info& type_info, const std::string& description,
	         const boost::any& default_value);

public:
	using type_info = boost::typeindex::type_info;

	/// Construct a property holding a any value
	Property();

	/// base class for Property exceptions
	class error : public std::runtime_error
	{
	protected:
		std::string property_name_;
		std::string msg_;

	public:
		explicit error(const std::string& msg);
		const std::string& name() const { return property_name_; }
		void setName(const std::string& name);
		const char* what() const noexcept override { return msg_.c_str(); }
	};

	/// exception thrown when accessing an undeclared property
	class undeclared : public Property::error
	{
	public:
		explicit undeclared(const std::string& name);
	};

	/// exception thrown when accessing an undefined property
	class undefined : public Property::error
	{
	public:
		explicit undefined();
		explicit undefined(const std::string& name);
	};

	/// exception thrown when trying to set a value not matching the declared type
	class type_error : public Property::error
	{
	public:
		explicit type_error(const std::string& current_type, const std::string& declared_type);
	};

	using SourceFlags = uint;
	/// function callback used to initialize property value from another PropertyMap
	using InitializerFunction = std::function<boost::any(const PropertyMap&)>;

	/// set current value and default value
	void setValue(const boost::any& value);
	void setCurrentValue(const boost::any& value);
	void setDefaultValue(const boost::any& value);

	/// reset to default value (which can be empty)
	void reset();

	/// is a value defined?
	inline bool defined() const { return !(value_.empty() && default_.empty()); }

	/// is a non-default value set?
	inline bool hasCurrentValue() const { return !value_.empty(); }

	/// get current value (or default if not set)
	inline const boost::any& value() const { return value_.empty() ? default_ : value_; }

	/// get typed value of property. Throws bad_any_cast on type mismatch, undefined if !defined().
	template <typename T>
	inline const T& value() const {
		const boost::any& v{ value() };
		if (v.empty())
			throw Property::undefined();
		return boost::any_cast<const T&>(value());
	}

	/// get default value
	const boost::any& defaultValue() const { return default_; }
	const boost::any& currentValue() const { return value_; }
	boost::any& currentValue() { return value_; };

	/// serialize value using registered functions
	static std::string serialize(const boost::any& value);
	static boost::any deserialize(const std::string& type_name, const std::string& wire);

	template <typename T>
	static T deserialize(const moveit_task_constructor_msgs::Property& property_msg) {
		PropertySerializer<T>();
		return boost::any_cast<T>(deserialize(property_msg.type, property_msg.value));
	}

	std::string serialize() const { return serialize(value()); }

	/// get description text
	const std::string& description() const { return description_; }
	void setDescription(const std::string& desc) { description_ = desc; }

	/// get typename
	static std::string typeName(const type_info& type_info);
	std::string typeName() const;

	/// return true, if property initialized from given SourceId
	bool initsFrom(SourceFlags source) const;
	/// configure initialization from source using an arbitrary function
	Property& configureInitFrom(SourceFlags source, const InitializerFunction& f);
	/// configure initialization from source using given other property name
	Property& configureInitFrom(SourceFlags source, const std::string& name);

	void fillMsg(moveit_task_constructor_msgs::Property& msg) const {
		msg.description = description();
		msg.type = typeName();
		msg.value = serialize();
	}

private:
	std::string description_;
	const type_info& type_info_;
	boost::any default_;
	boost::any value_;

	/// used for external initialization
	SourceFlags source_flags_ = 0;
	SourceFlags initialized_from_;
	InitializerFunction initializer_;
};

/** PropertyMap is map of (name, Property) pairs.
 *
 * Conveniency methods are provided to setup property initialization for several
 * properties at once - always inheriting from the identically named external property.
 */
class PropertyMap
{
	std::map<std::string, Property> props_;

	/// implementation of declare methods
	Property& declare(const std::string& name, const Property::type_info& type_info, const std::string& description,
	                  const boost::any& default_value);

public:
	/// declare a property for future use
	template <typename T>
	Property& declare(const std::string& name, const std::string& description = "") {
		PropertySerializer<T>();  // register serializer/deserializer
		return declare(name, typeid(T), description, boost::any());
	}
	/// declare a property with default value
	template <typename T>
	Property& declare(const std::string& name, const T& default_value, const std::string& description = "") {
		PropertySerializer<T>();  // register serializer/deserializer
		return declare(name, typeid(T), description, default_value);
	}

	/// declare all given properties also in other PropertyMap
	void exposeTo(PropertyMap& other, const std::set<std::string>& properties) const;

	/// declare given property name as other_name in other PropertyMap
	void exposeTo(PropertyMap& other, const std::string& name, const std::string& other_name) const;

	/// check whether given property is declared
	bool hasProperty(const std::string& name) const;

	/// get the property with given name, throws Property::undeclared for unknown name
	Property& property(const std::string& name);
	const Property& property(const std::string& name) const { return const_cast<PropertyMap*>(this)->property(name); }

	void fillMsgs(std::vector<moveit_task_constructor_msgs::Property>& msgs) const;
	void fromMsgs(std::vector<moveit_task_constructor_msgs::Property>& msgs);

	using iterator = std::map<std::string, Property>::iterator;
	using const_iterator = std::map<std::string, Property>::const_iterator;

	iterator begin() { return props_.begin(); }
	iterator end() { return props_.end(); }
	const_iterator begin() const { return props_.begin(); }
	const_iterator end() const { return props_.end(); }
	size_t size() const { return props_.size(); }

	/// allow initialization from given source for listed properties - always using the same name
	void configureInitFrom(Property::SourceFlags source, const std::set<std::string>& properties = {});

	/// set (and, if necessary, declare) the value of a property
	template <typename T>
	void set(const std::string& name, const T& value) {
		auto it = props_.find(name);
		if (it == props_.end())  // name is not yet declared
			declare<T>(name, value, "");
		else
			it->second.setValue(value);
	}

	/// overloading: const char* is stored as std::string
	inline void set(const std::string& name, const char* value) { set<std::string>(name, value); }

	/// temporarily set the value of a property
	void setCurrent(const std::string& name, const boost::any& value);

	/// Get the value of a property. Throws undeclared if unknown name
	const boost::any& get(const std::string& name) const;

	/// Get typed value of property. Throws undeclared or bad_any_cast.
	template <typename T>
	const T& get(const std::string& name) const {
		try {
			return property(name).value<T>();
		} catch (Property::undefined& e) {
			e.setName(name);
			throw e;
		}
	}

	/// count number of defined properties from given list
	size_t countDefined(const std::vector<std::string>& list) const;

	/// reset all properties to their defaults
	void reset();

	/// perform initialization of still undefined properties using configured initializers
	void performInitFrom(Property::SourceFlags source, const PropertyMap& other);
};

// boost::any needs a specialization to avoid infinite recursion
template <>
void PropertyMap::set<boost::any>(const std::string& name, const boost::any& value);

}  // namespace task_constructor
}  // namespace moveit
