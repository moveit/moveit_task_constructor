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
#include <boost/format.hpp>
#include <functional>

namespace moveit {
namespace task_constructor {

Property::Property(const std::type_index& type_index, const std::string& description, const boost::any& default_value)
   : description_(description)
   , type_index_(type_index)
   , default_(default_value)
   , value_(default_value)
{
	// default value's type should match declared type by construction
	assert(default_.empty() || std::type_index(default_.type()) == type_index_);
}

namespace {
void typeCheck(const boost::any& value, const std::type_index& type_index)
{
	if (std::type_index(value.type()) != type_index) {
		static boost::format fmt("type (%1%) doesn't match property's declared type (%2%)");
		throw std::logic_error(boost::str(fmt % value.type().name() % type_index.name()));
	}
}
}

void Property::setValue(const boost::any &value) {
	setCurrentValue(value);
	default_ = value_;
}

void Property::setCurrentValue(const boost::any &value)
{
	if (!value.empty())
		typeCheck(value, type_index_);
	value_ = value;
}

void Property::reset()
{
	value_ = default_;
}



Property& Property::configureInitFrom(SourceId source, const Property::InitializerFunction &f)
{
	if (source != source_id_ && initializer_)
		throw std::runtime_error("Property was already configured for initialization from another source id");

	source_id_ = source;
	initializer_ = f;
	return *this;
}

Property &Property::configureInitFrom(SourceId source, const std::string &name)
{
	return configureInitFrom(source, [name](const PropertyMap& other) { return fromName(other, name); });
}

void Property::performInitFrom(SourceId source, const PropertyMap &other)
{
	if (source_id_ != source || !initializer_) return;  // source ids not matching
	setCurrentValue(initializer_(other));
}


Property& PropertyMap::declare(const std::string &name, const std::type_info &type,
                               const std::string &description, const boost::any &default_value)
{
	auto it_inserted = props_.insert(std::make_pair(name, Property(std::type_index(type), description, default_value)));
	if (!it_inserted.second && std::type_index(type) != it_inserted.first->second.type_index_)
		throw std::logic_error("Property '" + name + "' was already declared with different type.");
	return it_inserted.first->second;
}

Property& PropertyMap::property(const std::string &name)
{
	auto it = props_.find(name);
	if (it == props_.end())
		throw std::runtime_error("Undeclared property '" + name + "'");
	return it->second;
}

void PropertyMap::configureInitFrom(Property::SourceId source, const std::set<std::string> &properties)
{
	for (auto &pair : props_) {
		if (properties.empty() || properties.count(pair.first))
			pair.second.configureInitFrom(source, std::bind(&fromName, std::placeholders::_1, pair.first));
	}
}

void PropertyMap::set(const std::string &name, const boost::any &value)
{
	auto range = props_.equal_range(name);
	if (range.first == range.second) { // name is not yet declared
		if (value.empty())
			throw std::logic_error("trying to set undeclared property '" + name + "' with NULL value");
		auto it = props_.insert(range.first, std::make_pair(name, Property(value.type(), "", boost::any())));
		it->second.setValue(value);
	} else {
		assert(range.first->first == name);
		range.first->second.setValue(value);
	}
}

void PropertyMap::setCurrent(const std::string &name, const boost::any &value)
{
	property(name).setCurrentValue(value);
}

const boost::any &PropertyMap::get(const std::string &name) const
{
	return property(name).value();
}

size_t PropertyMap::countDefined(const std::vector<std::string> &list) const
{
	size_t count = 0u;
	for (const std::string& name : list) {
		if (!get(name).empty()) ++count;
	}
	return count;
}

void PropertyMap::reset()
{
	for (auto& pair : props_)
		pair.second.reset();
}

void PropertyMap::performInitFrom(Property::SourceId source, const PropertyMap &other, bool enforce)
{
	for (auto& pair : props_) {
		Property &p = pair.second;
		if (enforce || !p.defined())
			p.performInitFrom(source, other);
	}
}


boost::any fromName(const PropertyMap& other, const std::string& other_name)
{
	try {
		return other.get(other_name);
	} catch (const std::runtime_error &e) {
		return boost::any();
	}
}

} // namespace task_constructor
} // namespace moveit
