#pragma once

#include <moveit/macros/class_forward.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}
}  // namespace moveit

// get a hard-coded model
moveit::core::RobotModelPtr getModel();
