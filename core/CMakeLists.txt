cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_core)

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(py_binding_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_marker_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-fvisibility-inlines-hidden)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/task_constructor)

add_subdirectory(src)
ament_python_install_package(moveit PACKAGE_DIR python/moveit)
add_subdirectory(python)
add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include
	PATTERN "*_p.h" EXCLUDE)

pluginlib_export_plugin_description_file(${PROJECT_NAME} motion_planning_stages_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_task_constructor_msgs)
ament_export_dependencies(py_binding_tools)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_marker_tools)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(visualization_msgs)
ament_package()
