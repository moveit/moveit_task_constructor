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
		const boost::any& value = get(name);
		if (value.empty())
			throw std::runtime_error(std::string("undefined property: " + name));
		return boost::any_cast<const T&>(value);
	}

	/// count number of defined properties from given list
	size_t countDefined(const std::vector<std::string>& list) const;

	/// reset properties to nil, if they have initializers
	void reset();
	/// init properties from initializers
	void initFrom(PropertyInitializerSource source, const PropertyMap& other,
	              bool checkConsistency = false);
};

} // namespace task_constructor
} // namespace moveit
