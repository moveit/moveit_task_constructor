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

#include <moveit/task_constructor/properties.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/clock.hpp>

namespace moveit {
namespace task_constructor {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Properties");

class PropertyTypeRegistry
{
	struct Entry
	{
		std::string name_;
		PropertySerializerBase::SerializeFunction serialize_;
		PropertySerializerBase::DeserializeFunction deserialize_;
	};
	Entry dummy_;

	// map from type_info to corresponding converter functions
	using RegistryMap = std::map<std::type_index, Entry>;
	RegistryMap types_;
	// map from type names (type.name or ROS msg name) to entry in types_
	using TypeNameMap = std::map<std::string, RegistryMap::iterator>;
	TypeNameMap names_;

public:
	PropertyTypeRegistry()
	  : dummy_{ "", PropertySerializerBase::dummySerialize, PropertySerializerBase::dummyDeserialize } {}
	inline bool insert(const std::type_index& type_index, const std::string& type_name,
	                   PropertySerializerBase::SerializeFunction serialize,
	                   PropertySerializerBase::DeserializeFunction deserialize);

	const Entry& entry(const std::type_index& type_index) const {
		auto it = types_.find(type_index);
		if (it == types_.end()) {
			rclcpp::Clock steady_clock(RCL_STEADY_TIME);
			RCLCPP_WARN_STREAM_THROTTLE(LOGGER, steady_clock, 10'000,
			                            "Unregistered property type: " << boost::core::demangle(type_index.name()));
			return dummy_;
		}
		return it->second;
	}
	const Entry& entry(const std::string& type_name) const {
		auto it = names_.find(type_name);
		if (it == names_.end())
			return dummy_;
		return it->second->second;
	}
};
static PropertyTypeRegistry REGISTRY_SINGLETON;

bool PropertyTypeRegistry::insert(const std::type_index& type_index, const std::string& type_name,
                                  PropertySerializerBase::SerializeFunction serialize,
                                  PropertySerializerBase::DeserializeFunction deserialize) {
	if (type_index == std::type_index(typeid(boost::any)))
		return false;

	auto it_inserted = types_.insert(std::make_pair(type_index, Entry{ type_name, serialize, deserialize }));
	if (!it_inserted.second)
		return false;  // was already registered before

	if (!type_name.empty())  // register type_name too?
		names_.insert(std::make_pair(type_name, it_inserted.first));

	return true;
}

bool PropertySerializerBase::insert(const std::type_index& type_index, const std::string& type_name,
                                    PropertySerializerBase::SerializeFunction serialize,
                                    PropertySerializerBase::DeserializeFunction deserialize) {
	return REGISTRY_SINGLETON.insert(type_index, type_name, serialize, deserialize);
}

Property::Property(const type_info& type_info, const std::string& description, const boost::any& default_value)
  : description_(description), type_info_(type_info), default_(default_value), value_(), initialized_from_(-1) {
	// default value's type should match declared type by construction
	assert(default_.empty() || default_.type() == type_info_ || type_info_ == typeid(boost::any));
	reset();
}

Property::Property() : Property(typeid(boost::any), "", boost::any()) {}

void Property::setValue(const boost::any& value) {
	setCurrentValue(value);
	default_ = value_;
	initialized_from_ = 0;
}

void Property::setCurrentValue(const boost::any& value) {
	if (!value.empty() && type_info_ != typeid(boost::any) && value.type() != type_info_)
		throw Property::type_error(value.type().name(), type_info_.name());

	value_ = value;
	initialized_from_ = 1;  // manually initialized TODO: use enums
}

void Property::setDefaultValue(const boost::any& value) {
	if (!value.empty() && type_info_ != typeid(boost::any) && value.type() != type_info_)
		throw Property::type_error(value.type().name(), type_info_.name());

	default_ = value;
}

void Property::reset() {
	if (initialized_from_ == 0)  // TODO: use enum
		return;  // keep manually set values
	boost::any().swap(value_);
	initialized_from_ = -1;  // set to max value
}

std::string Property::serialize(const boost::any& value) {
	if (value.empty())
		return "";
	return REGISTRY_SINGLETON.entry(value.type()).serialize_(value);
}

boost::any Property::deserialize(const std::string& type_name, const std::string& wire) {
	if (type_name != Property::typeName(typeid(std::string)) && wire.empty())
		return boost::any();
	else
		return REGISTRY_SINGLETON.entry(type_name).deserialize_(wire);
}

std::string Property::typeName() const {
	if (value().empty())
		return typeName(type_info_);
	else
		return typeName(value().type());
}

std::string Property::typeName(const type_info& type_info) {
	if (type_info == typeid(boost::any))
		return "";
	else
		return REGISTRY_SINGLETON.entry(type_info).name_;
}

bool Property::initsFrom(Property::SourceFlags source) const {
	return source & source_flags_;
}

Property& Property::configureInitFrom(SourceFlags source, const Property::InitializerFunction& f) {
	if (source != source_flags_ && initializer_)
		throw error("Property was already configured for initialization from another source id");

	source_flags_ = f ? source : SourceFlags();
	initializer_ = f;
	return *this;
}

Property& Property::configureInitFrom(SourceFlags source, const std::string& name) {
	return configureInitFrom(source, [name](const PropertyMap& other) { return fromName(other, name); });
}

Property& PropertyMap::declare(const std::string& name, const Property::type_info& type_info,
                               const std::string& description, const boost::any& default_value) {
	auto it_inserted = props_.insert(std::make_pair(name, Property(type_info, description, default_value)));
	// if name was already declared, the new declaration should match in type (except it was boost::any)
	if (!it_inserted.second && it_inserted.first->second.type_info_ != typeid(boost::any) &&
	    type_info != it_inserted.first->second.type_info_)
		throw Property::type_error(type_info.name(), it_inserted.first->second.type_info_.name());
	return it_inserted.first->second;
}

bool PropertyMap::hasProperty(const std::string& name) const {
	auto it = props_.find(name);
	return it != props_.end();
}

Property& PropertyMap::property(const std::string& name) {
	auto it = props_.find(name);
	if (it == props_.end())
		throw Property::undeclared(name);
	return it->second;
}

void PropertyMap::exposeTo(PropertyMap& other, const std::set<std::string>& properties) const {
	for (const std::string& name : properties)
		exposeTo(other, name, name);
}

void PropertyMap::exposeTo(PropertyMap& other, const std::string& name, const std::string& other_name) const {
	const Property& p = property(name);
	other.declare(other_name, p.type_info_, p.description_, p.default_);
}

void PropertyMap::configureInitFrom(Property::SourceFlags source, const std::set<std::string>& properties) {
	for (auto& pair : props_) {
		if (properties.empty() || properties.count(pair.first))
			try {
				pair.second.configureInitFrom(source, std::bind(&fromName, std::placeholders::_1, pair.first));
			} catch (Property::error& e) {
				e.setName(pair.first);
				throw;
			}
	}
}

template <>
void PropertyMap::set<boost::any>(const std::string& name, const boost::any& value) {
	auto range = props_.equal_range(name);
	if (range.first == range.second) {  // name is not yet declared
		if (value.empty())
			throw Property::undeclared(name, "trying to set undeclared property '" + name + "' with NULL value");
		auto it = props_.insert(range.first, std::make_pair(name, Property(value.type(), "", boost::any())));
		it->second.setValue(value);
	} else
		range.first->second.setValue(value);
}

void PropertyMap::setCurrent(const std::string& name, const boost::any& value) {
	property(name).setCurrentValue(value);
}

const boost::any& PropertyMap::get(const std::string& name) const {
	return property(name).value();
}

size_t PropertyMap::countDefined(const std::vector<std::string>& list) const {
	size_t count = 0u;
	for (const std::string& name : list) {
		if (!get(name).empty())
			++count;
	}
	return count;
}

void PropertyMap::reset() {
	for (auto& pair : props_)
		pair.second.reset();
}

void PropertyMap::performInitFrom(Property::SourceFlags source, const PropertyMap& other) {
	for (auto& pair : props_) {
		Property& p = pair.second;

		// don't override value previously set by higher-priority source
		// MANUAL > CURRENT > PARENT > INTERFACE
		if (p.initialized_from_ < source && p.defined())
			continue;
		// is the property configured for initialization from this source?
		if (!p.initsFrom(source))
			continue;

		boost::any value;
		try {
			value = p.initializer_(other);
		} catch (const Property::undeclared&) {
			// ignore undeclared
			continue;
		} catch (const Property::undefined&) {
		}

		RCLCPP_DEBUG_STREAM(
		    LOGGER, fmt::format("{}: {} -> {}: {}", pair.first, p.initialized_from_, source, Property::serialize(value)));
		p.setCurrentValue(value);
		p.initialized_from_ = source;
	}
}

boost::any fromName(const PropertyMap& other, const std::string& other_name) {
	return other.get(other_name);
}

Property::error::error(const std::string& msg) : std::runtime_error(msg), msg_("Property: " + msg) {}

void Property::error::setName(const std::string& name) {
	property_name_ = name;
	// compose message from property name and runtime_errors' msg
	msg_ = "Property '" + name + "': " + std::runtime_error::what();
}

Property::undeclared::undeclared(const std::string& name, const std::string& msg) : Property::error(msg) {
	setName(name);
}

Property::undefined::undefined(const std::string& name, const std::string& msg) : Property::error(msg) {
	setName(name);
}

Property::type_error::type_error(const std::string& current_type, const std::string& declared_type)
  : Property::error(fmt::format("type {} doesn't match property's declared type {}", current_type, declared_type)) {}

}  // namespace task_constructor
}  // namespace moveit
