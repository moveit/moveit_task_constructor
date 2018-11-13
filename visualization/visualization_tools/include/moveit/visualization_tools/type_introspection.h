#pragma once

#include <ros/message_traits.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <ros_type_introspection/ros_introspection.hpp>

#include <moveit_task_constructor_msgs/Property.h>

namespace ri = RosIntrospection;

namespace moveit_rviz_plugin {

class TreeConstructor {

public:
	static rviz::Property* parseToTree(const ri::RenamedValues& msgValues,
	                                   const std::string& name, const std::string& description,
	                                   rviz::Property* old = 0);

private:
	static void treeFromValue(const std::pair<std::string, ri::Variant>& value,
	                          rviz::Property* root, const size_t startIx);

	static rviz::Property* getOrCreateChild(const std::string& name, rviz::Property* parent);

	static rviz::Property* createLeaf(const std::pair<std::string, ri::Variant>& value,
	                                  rviz::Property* parent, const size_t startIx);

	template <typename T, typename V>
	static rviz::Property* createLeafOfType(const std::pair<std::string, ri::Variant>& value,
	                                        rviz::Property* parent, const size_t startIx);

	template <typename V>
	static rviz::StringProperty* createStringLeaf(const std::pair<std::string, ri::Variant>& value,
	                                              rviz::Property* parent, const size_t startIx);

	static rviz::StringProperty* timeLeaf(const std::pair<std::string, ri::Variant>& value,
	                                      rviz::Property* parent, const size_t startIx);

	static rviz::StringProperty* durationLeaf(const std::pair<std::string, ri::Variant>& value,
	                                          rviz::Property* parent, const size_t startIx);
};


class TypeIntrospector {

public:
	/// Return singleton instance
	inline static TypeIntrospector& instance() {
		static TypeIntrospector instance_;
		return instance_;
	}

	/// Register the given type to the parser. Registering the same message twice is allowed.
	template <typename T>
	inline void registerMsgType() {
		parser.registerMessageDefinition(ros::message_traits::datatype<T>(),
		                                 ri::ROSType(ros::message_traits::datatype<T>()),
		                                 ros::message_traits::definition<T>());
	}

	/// Return extracted values of the message contained in the given property.
	inline ri::RenamedValues extractFromPropertyMsg(const moveit_task_constructor_msgs::Property& prop) {
		return extract(prop.type, prop.value);
	}

	/** Return extracted values of a serialized message as a flat vector.
	 * `name` is the name by which the message was registered, while
	 * `sermsg` is the serialized message. */
	ri::RenamedValues extract(const std::string& msg_id, const std::string& serialized);

	/** Return extracted values of a serialized msg in a buffer as a flat vector.
	 * `name` is the name by which the message was registered, while
	 * `sermsg` is a buffer containing the serialized message. */
	ri::RenamedValues extract(const std::string& name, std::vector<uint8_t>& serialized);

private:
	TypeIntrospector() {}

	/// The parser containing all registered message types.
	ri::Parser parser;
};

} // namespace moveit_rviz_plugin
