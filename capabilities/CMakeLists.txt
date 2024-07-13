cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_capabilities)

find_package(ament_cmake REQUIRED)

find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
   src/execute_task_solution_capability.cpp
)
ament_target_dependencies(${PROJECT_NAME}
                          Boost
                          fmt
                          rclcpp_action
                          moveit_core
                          moveit_ros_move_group
                          moveit_ros_planning
                          moveit_task_constructor_core
                          moveit_task_constructor_msgs
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(moveit_ros_move_group capabilities_plugin_description.xml)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_move_group)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_task_constructor_core)
ament_export_dependencies(moveit_task_constructor_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(std_msgs)
ament_package()
