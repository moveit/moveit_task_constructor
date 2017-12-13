#include <moveit/task_constructor/properties.h>
#include <boost/format.hpp>
#include <ros/console.h>

namespace moveit {
namespace task_constructor {

Property::Property(const std::type_index& type_index, const std::string& description, const boost::any& default_value)
   : description_(description)
   , type_index_(type_index)
   , default_(default_value)
{
	if (!default_.empty() && std::type_index(default_.type()) != type_index_)
		throw std::runtime_error("type of default value doesn't match declared type");
}

namespace {
void typeCheck(const boost::any& value, const std::type_index& type_index)
{
	if (std::type_index(value.type()) != type_index) {
		static boost::format fmt("type (%1%) doesn't match property's declared type (%2%)");
		throw std::runtime_error(boost::str(fmt % value.type().name() % type_index.name()));
	}
}
}

void Property::setValue(const boost::any &value) {
	typeCheck(value, type_index_);
	value_ = value;
	// clear all initializers when value was explicitly set
	initializers_.clear();
}

const boost::any &Property::value() const {
	return value_.empty() ? default_ : value_;
}

Property& Property::initFrom(PropertyInitializerSource source, const Property::InitializerFunction &f)
{
	initializers_[source] = f;
	return *this;
}

Property& Property::initFrom(PropertyInitializerSource source, const std::string &other_name)
{
	initializers_[source] = [other_name](const PropertyMap& other, const std::string&) {
		return fromName(other, other_name);
	};
	return *this;
}


Property& PropertyMap::declare(const std::string &name, const std::type_info &type,
                               const std::string &description, const boost::any &default_value)
{
	auto it_inserted = props_.insert(std::make_pair(name, Property(std::type_index(type), description, default_value)));
	if (!it_inserted.second && std::type_index(type) != it_inserted.first->second.type_index_)
		throw std::runtime_error("Property '" + name + "' was already declared with different type.");
	return it_inserted.first->second;
}

Property& PropertyMap::property(const std::string &name)
{
	auto it = props_.find(name);
	if (it == props_.end())
		throw std::runtime_error("Unknown property '" + name + "'");
	return it->second;
}

void PropertyMap::initFrom(PropertyInitializerSource source, const std::set<std::string> &properties)
{
	for (auto &pair : props_) {
		if (properties.empty() || properties.count(pair.first))
			pair.second.initFrom(source);
	}
}

void PropertyMap::set(const std::string &name, const boost::any &value)
{
	auto range = props_.equal_range(name);
	if (range.first == range.second) { // name is not yet declared
		if (value.empty()) {
			ROS_ERROR("setting undefined property '%s': valid value expected", name.c_str());
			return;
		}
		auto it = props_.insert(range.first, std::make_pair(name, Property(value.type(), "", boost::any())));
		it->second.setValue(value);
	} else {
		assert(range.first->first == name);
		range.first->second.setValue(value);
	}
}

const boost::any &PropertyMap::get(const std::string &name) const
{
	return property(name).value();
}

void PropertyMap::reset()
{
	for (auto& pair : props_) {
		Property &p = pair.second;
		if (!p.initializers_.empty())
			// if there are initializers, reset property value
			// explicitly setting the value, clears initializers
			p.value_ = boost::any();
	}
}

void PropertyMap::initFrom(PropertyInitializerSource source, const PropertyMap &other,
                           bool checkConsistency)
{
	for (auto& pair : props_) {
		Property &p = pair.second;
		auto it = p.initializers_.find(source);
		if (it == p.initializers_.end()) continue;
		const boost::any& value = it->second(other, pair.first);
		if (value.empty()) continue;

		typeCheck(value, p.type_index_);
#if 0 // TODO: we cannot generically check for equality of boost::anys
		if (checkConsistency && !p.value_.empty() && p.value_ != value)
			throw std::runtime_error ("inconsistent values for property '" + pair.first + "'");
#endif
		p.value_ = value;
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
