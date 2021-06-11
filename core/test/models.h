#pragma once

#include <moveit/macros/class_forward.h>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel);
}
}  // namespace moveit

// get a hard-coded model
moveit::core::RobotModelPtr getModel();

// load a model from robot_description
moveit::core::RobotModelPtr loadModel();
