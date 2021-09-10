/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2021, Universitaet Hamburg
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

/* Author: Robert Haschke / Michael 'v4hn' Goerner
   Desc:   Serialization/Deserialization support for Properties
*/

#pragma once

#include <boost/any.hpp>
#include <typeindex>
#include <type_traits>
#include <map>
#include <set>
#include <vector>
#include <functional>
#include <sstream>
#include <ros/serialization.h>

#include <moveit_task_constructor_msgs/Property.h>

namespace moveit {
namespace task_constructor {

class Property;

// has*Operator<T>::value is true iff operator<< resp. operator>> is available for T.
// This uses SFINAE, extracted from https://jguegant.github.io/blogs/tech/sfinae-introduction.html
template <typename T, typename = std::ostream&>
struct IsStreamSerializable : std::false_type
{};

template <typename T>
struct IsStreamSerializable<T, decltype(std::declval<std::ostream&>() << std::declval<T>())> : std::true_type
{};

template <typename T, typename = std::istream&>
struct IsStreamDeserializable : std::false_type
{};

template <typename T>
struct IsStreamDeserializable<T, decltype(std::declval<std::istream&>() >> std::declval<T&>())> : std::true_type
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

/** Serialization for std::string **/
class PropertySerializerString : public PropertySerializerBase
{
public:
	static const char* typeName() { return typeid(std::string).name(); }

	static std::string serialize(const boost::any& value) { return boost::any_cast<std::string>(value); }
	static boost::any deserialize(const std::string& wired) { return wired; }
};

/** Serialization for ROS messages **/
template <typename T>
class PropertySerializerROS : public PropertySerializerBase
{
public:
	static const char* typeName() { return ros::message_traits::DataType<T>::value(); }

	static std::string serialize(const boost::any& value) {
		T message{ boost::any_cast<T>(value) };
		uint32_t serial_size = ros::serialization::serializationLength(message);
		std::string buffer(serial_size, '\0');
		ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(&buffer.front()), serial_size);
		ros::serialization::serialize(stream, message);
		return buffer;
	}

	static boost::any deserialize(const std::string& wired) {
		ros::serialization::IStream stream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&wired.front())),
		                                   wired.size());
		T message;
		ros::serialization::deserialize(stream, message);
		return message;
	}
};

/** Serialization based on std::[io]stringstream */
template <typename T>
class PropertySerializerStream : public PropertySerializerBase
{
public:
	static const char* typeName() { return typeid(T).name(); }

	template <typename Q = T>
	static std::enable_if_t<IsStreamSerializable<Q>::value, std::string> serialize(const boost::any& value) {
		std::ostringstream oss;
		oss << boost::any_cast<T>(value);
		return oss.str();
	}
	template <typename Q = T>
	static std::enable_if_t<IsStreamDeserializable<Q>::value, boost::any> deserialize(const std::string& wired) {
		std::istringstream iss(wired);
		T value;
		iss >> value;
		return value;
	}

	/** fallback, if no serialization/deserialization is available **/
	template <typename Q = T>
	static std::enable_if_t<!IsStreamSerializable<Q>::value, std::string> serialize(const boost::any& value) {
		return dummySerialize(value);
	}
	template <typename Q = T>
	static std::enable_if_t<!IsStreamDeserializable<Q>::value, boost::any> deserialize(const std::string& wire) {
		return dummyDeserialize(wire);
	}
};

template <typename T>
class PropertySerializer
  : public std::conditional_t<std::is_same<T, std::string>::value, PropertySerializerString,
                              std::conditional_t<ros::message_traits::IsMessage<T>::value, PropertySerializerROS<T>,
                                                 PropertySerializerStream<T>>>
{
public:
	PropertySerializer() {
		this->insert(typeid(T), this->typeName(), &PropertySerializer::serialize, &PropertySerializer::deserialize);
	}
};

}  // namespace task_constructor
}  // namespace moveit
