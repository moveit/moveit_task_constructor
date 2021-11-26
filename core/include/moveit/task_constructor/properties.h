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
#include <rclcpp/serialization.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

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
	class error;
	/// exception thrown when accessing an undeclared property
	class undeclared;
	/// exception thrown when accessing an undefined property
	class undefined;
	/// exception thrown when trying to set a value not matching the declared type
	class type_error;

	using SourceFlags = uint;
	/// function callback used to initialize property value from another PropertyMap
	using InitializerFunction = std::function<boost::any(const PropertyMap&)>;

	/// set current value and default value
	void setValue(const boost::any& value);
	void setCurrentValue(const boost::any& value);
	void setDefaultValue(const boost::any& value);

	/// reset to default value (which can be empty)
	void reset();

	/// the current value defined or will the fallback be used?
	inline bool defined() const { return !value_.empty(); }

	/// get current value (or default if not defined)
	inline const boost::any& value() const { return value_.empty() ? default_ : value_; }
	inline boost::any& value() { return value_.empty() ? default_ : value_; }
	/// get default value
	const boost::any& defaultValue() const { return default_; }

	/// serialize value using registered functions
	static std::string serialize(const boost::any& value);
	static boost::any deserialize(const std::string& type_name, const std::string& wire);
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

class Property::error : public std::runtime_error
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
class Property::undeclared : public Property::error
{
public:
	explicit undeclared(const std::string& name, const std::string& msg = "undeclared");
};
class Property::undefined : public Property::error
{
public:
	explicit undefined(const std::string& name, const std::string& msg = "undefined");
};
class Property::type_error : public Property::error
{
public:
	explicit type_error(const std::string& current_type, const std::string& declared_type);
};

// hasSerialize<T>::value provides a true/false constexpr depending on whether operator<< is supported.
// This uses SFINAE, extracted from https://jguegant.github.io/blogs/tech/sfinae-introduction.html
template <typename T, typename = std::ostream&>
struct hasSerialize : std::false_type
{};

template <typename T>
struct hasSerialize<T, decltype(std::declval<std::ostream&>() << std::declval<T>())> : std::true_type
{};

template <typename T, typename = std::istream&>
struct hasDeserialize : std::false_type
{};

template <typename T>
struct hasDeserialize<T, decltype(std::declval<std::istream&>() >> std::declval<T&>())> : std::true_type
{};

class PropertySerializerBase
{
public:
	using SerializeFunction = std::string (*)(const boost::any&);
	using DeserializeFunction = boost::any (*)(const std::string&);

	static std::string dummySerialize(const boost::any& /*unused*/) { return ""; }
	static boost::any dummyDeserialize(const std::string& /*unused*/) { return boost::any(); }

protected:
	static bool insert(const std::type_index& type_index, const std::string& type_name, SerializeFunction serialize,
	                   DeserializeFunction deserialize);
};

/// utility class to register serializer/deserializer functions for a property of type T
template <typename T>
class PropertySerializer : protected PropertySerializerBase
{
public:
	PropertySerializer() { insert(typeid(T), typeName<T>(), &serialize, &deserialize); }

	template <class Q = T>
	static typename std::enable_if<rosidl_generator_traits::is_message<Q>::value, std::string>::type typeName() {
		return rosidl_generator_traits::data_type<T>();
	}

	template <class Q = T>
	static typename std::enable_if<!rosidl_generator_traits::is_message<Q>::value, std::string>::type typeName() {
		return typeid(T).name();
	}

private:
	/** Serialization based on std::[io]stringstream */
	template <class Q = T>
	static typename std::enable_if<hasSerialize<Q>::value, std::string>::type serialize(const boost::any& value) {
		std::ostringstream oss;
		oss << boost::any_cast<T>(value);
		return oss.str();
	}
	template <class Q = T>
	static typename std::enable_if<hasSerialize<Q>::value && hasDeserialize<Q>::value, boost::any>::type
	deserialize(const std::string& wired) {
		std::istringstream iss(wired);
		T value;
		iss >> value;
		return value;
	}

	/** No serialization available */
	template <class Q = T>
	static typename std::enable_if<!hasSerialize<Q>::value, std::string>::type serialize(const boost::any& value) {
		return dummySerialize(value);
	}
	template <class Q = T>
	static typename std::enable_if<!hasSerialize<Q>::value || !hasDeserialize<Q>::value, boost::any>::type
	deserialize(const std::string& wire) {
		return dummyDeserialize(wire);
	}
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

	using iterator = std::map<std::string, Property>::iterator;
	using const_iterator = std::map<std::string, Property>::const_iterator;

	iterator begin() { return props_.begin(); }
	iterator end() { return props_.end(); }
	const_iterator begin() const { return props_.begin(); }
	const_iterator end() const { return props_.end(); }

	/// allow initialization from given source for listed properties - always using the same name
	void configureInitFrom(Property::SourceFlags source, const std::set<std::string>& properties = {});

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
	inline void set(const std::string& name, const char* value) { set<std::string>(name, value); }

	/// temporarily set the value of a property
	void setCurrent(const std::string& name, const boost::any& value);

	/// Get the value of a property. Throws undeclared if unknown name
	const boost::any& get(const std::string& name) const;

	/// Get typed value of property. Throws undeclared, undefined, or bad_any_cast.
	template <typename T>
	const T& get(const std::string& name) const {
		const boost::any& value = get(name);
		if (value.empty())
			throw Property::undefined(name);
		return boost::any_cast<const T&>(value);
	}
	/// get typed value of property, using fallback if undefined. Throws bad_any_cast on type mismatch.
	template <typename T>
	const T& get(const std::string& name, const T& fallback) const {
		const boost::any& value = get(name);
		return (value.empty()) ? fallback : boost::any_cast<const T&>(value);
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
