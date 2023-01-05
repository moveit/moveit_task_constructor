#pragma once

#include "ros_types.h"
#include <geometry_msgs/PoseStamped.h>

/** Convienency type casters, also allowing to initialize Stamped geometry msgs from a string */

namespace pybind11 {
namespace detail {

template <>
struct type_caster<geometry_msgs::PoseStamped> : type_caster_ros_msg<geometry_msgs::PoseStamped>
{
	// Python -> C++
	bool load(handle src, bool convert) {
		type_caster<std::string> str_caster;
		if (convert && str_caster.load(src, false)) {  // string creates identity pose with given frame
			value.header.frame_id = static_cast<std::string&>(str_caster);
			value.pose.orientation.w = 1.0;
			return true;
		}
		return type_caster_ros_msg<geometry_msgs::PoseStamped>::load(src, convert);
	}
};

}  // namespace detail
}  // namespace pybind11
