#include <moveit/task_constructor/stages/current_state.h>

#include <pluginlib/class_list_macros.hpp>

/// register plugins to use with ClassLoader

PLUGINLIB_EXPORT_CLASS(moveit::task_constructor::stages::CurrentState, moveit::task_constructor::Stage)
