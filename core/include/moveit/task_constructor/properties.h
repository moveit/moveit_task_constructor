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
#include <functional>

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

public:
	typedef int SourceId;
	typedef std::function<boost::any(const PropertyMap& other)> InitializerFunction;
	typedef std::map<SourceId, InitializerFunction> InitializerMap;

	Property(const std::type_index& type_index, const std::string& description, const boost::any& default_value);

	/// set current value and default value
	void setValue(const boost::any& value);
	void setCurrentValue(const boost::any& value);

	/// reset to default value (which can be empty)
	void reset();

	inline bool defined() const { return !value_.empty(); }

	/// get current value
	inline const boost::any& value() const { return value_; }
	/// get default value
	const boost::any& defaultValue() const { return default_; }

	/// get description text
	const std::string& description() const { return description_; }
	/// get typename
	std::string typeName() const { return type_index_.name(); }

	/// configure initialization from source using an arbitrary function
	Property &configureInitFrom(SourceId source, const InitializerFunction& f);
	/// configure initialization from source using given other property name
	Property &configureInitFrom(SourceId source, const std::string& name);

	/// set current value using configured initializers
	void performInitFrom(SourceId source, const PropertyMap& other);

private:
	std::string description_;
	std::type_index type_index_;
	boost::any default_;
	boost::any value_;
	InitializerMap initializers_;
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
	Property& declare(const std::string& name, const std::type_info& type,
	                  const std::string& description = "",
	                  const boost::any& default_value = boost::any());
public:
	/// declare a property for future use
	template<typename T>
	Property& declare(const std::string& name, const std::string& description = "") {
		return declare(name, typeid(T), description);
	}
	/// declare a property with default value
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

	/// allow initialization from given source for listed properties - always using the same name
	void configureInitFrom(Property::SourceId source, const std::set<std::string> &properties = {});

	/// set (and, if neccessary, declare) the value of a property
	void set(const std::string& name, const boost::any& value);
	/// temporarily set the value of a property
	void setCurrent(const std::string& name, const boost::any& value);

	/// get the value of a property
	const boost::any& get(const std::string& name) const;

	/// Get typed value of property. Throws runtime_error if undefined or bad_any_cast on type mismatch.
	template<typename T>
	const T& get(const std::string& name) const {
		const boost::any& value = get(name);
		if (value.empty())
			throw std::runtime_error(std::string("undefined property: " + name));
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

} // namespace task_constructor
} // namespace moveit
