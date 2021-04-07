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
#include <map>
#include <set>
#include <vector>
#include <functional>
#include <sstream>
#include <ros/serialization.h>

#include <moveit_task_constructor_msgs/Property.h>

#include "base64.hpp"

namespace moveit {
namespace task_constructor {

class Property;

// hasSerialize<T>::value provides a true/false constexpr depending on whether operator<< is supported.
// This uses SFINAE, extracted from https://jguegant.github.io/blogs/tech/sfinae-introduction.html
template <typename T, typename = std::ostream&>
struct hasInsertionOperator : std::false_type
{};

template <typename T>
struct hasInsertionOperator<T, decltype(std::declval<std::ostream&>() << std::declval<T>())> : std::true_type
{};

template <typename T>
constexpr bool hasInsertionOperator_v = hasInsertionOperator<T>::value;

template <typename T, typename = std::istream&>
struct hasExtractionOperator : std::false_type
{};

template <typename T>
struct hasExtractionOperator<T, decltype(std::declval<std::istream&>() >> std::declval<T&>())> : std::true_type
{};

template <typename T>
constexpr bool hasExtractionOperator_v = hasExtractionOperator<T>::value;

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

template <typename T>
class PropertySerializer : public PropertySerializerBase
{
protected:
	/// a set of mutually exclusive conditions
	template <typename U>
	static constexpr bool IsROSMessage{ ros::message_traits::IsMessage<U>::value };

	template <typename U>
	static constexpr bool IsStreamSerializable{ !IsROSMessage<U> && hasInsertionOperator_v<U> };

	template <typename U>
	static constexpr bool IsStreamDeserializable{ !IsROSMessage<U> && hasExtractionOperator_v<U> };

	template <typename U>
	static constexpr bool IsString{ std::is_same<U, std::string>::value };

	template <typename U>
	static constexpr bool IsNotDeserializable{ !IsROSMessage<U> && !IsStreamSerializable<U> };

public:
	PropertySerializer() { this->insert(typeid(T), this->typeName<T>(), &serialize, &deserialize); }

	template <typename Q = T>
	static std::enable_if_t<!IsROSMessage<Q>, std::string> typeName() {
		return typeid(T).name();
	}

	template <typename Q = T>
	static std::enable_if_t<IsROSMessage<Q>, std::string> typeName() {
		return ros::message_traits::DataType<T>::value();
	}

	/** ROS messages are serialized using the provided binary encoding */
	template <typename Q = T>
	static std::enable_if_t<IsROSMessage<Q>, std::string> serialize(const boost::any& value) {
		Q message{ boost::any_cast<Q>(value) };
		uint32_t serial_size = ros::serialization::serializationLength(message);
		std::unique_ptr<uint8_t[]> buffer(new uint8_t[serial_size]);
		ros::serialization::OStream stream(buffer.get(), serial_size);
		ros::serialization::serialize(stream, message);
		return reinterpret_cast<const char*>(base64::encode(buffer.get(), serial_size).c_str());
	}
	template <typename Q = T>
	static std::enable_if_t<IsROSMessage<Q>, boost::any> deserialize(const std::string& wired) {
		auto decoded{ base64::decode(wired.data(), wired.size()) };
		ros::serialization::IStream stream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(decoded.data())),
		                                   wired.size());
		Q message;
		ros::serialization::deserialize(stream, message);
		return message;
	}

	/** Serialization based on std::[io]stringstream */
	template <typename Q = T>
	static std::enable_if_t<IsStreamSerializable<Q>, std::string> serialize(const boost::any& value) {
		std::ostringstream oss;
		oss << boost::any_cast<T>(value);
		return oss.str();
	}
	template <typename Q = T>
	static std::enable_if_t<IsStreamDeserializable<Q> && !IsString<Q>, boost::any>
	deserialize(const std::string& wired) {
		std::istringstream iss(wired);
		T value;
		iss >> value;
		return value;
	}
	// (istream >> string) only extracts until first whitespace, so we specialize
	template <typename Q = T>
	static std::enable_if_t<IsString<Q>, boost::any> deserialize(const std::string& wired) {
		return wired;
	}

	/** No serialization available */
	template <typename Q = T>
	static std::enable_if_t<IsNotDeserializable<Q>, std::string> serialize(const boost::any& value) {
		return dummySerialize(value);
	}

	template <typename Q = T>
	static std::enable_if_t<IsNotDeserializable<Q>, boost::any> deserialize(const std::string& wire) {
		return dummyDeserialize(wire);
	}
};
}  // namespace task_constructor
}  // namespace moveit
