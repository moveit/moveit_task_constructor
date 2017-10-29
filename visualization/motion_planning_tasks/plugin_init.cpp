/* Author: Robert Haschke */

#include <pluginlib/class_list_macros.h>
#include "task_display.h"
#include "task_panel.h"

PLUGINLIB_EXPORT_CLASS(moveit_rviz_plugin::TaskDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(moveit_rviz_plugin::TaskPanel, rviz::Panel)
