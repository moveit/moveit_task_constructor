cmake_minimum_required(VERSION 3.16)
project(moveit_task_constructor_visualization)

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

# Qt Stuff: inherit major version from rviz_common, which find-packages Qt
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
elseif(Qt5_FOUND)
  set(QT_VERSION_MAJOR 5)
endif()

set(QT_LIBRARIES Qt${QT_VERSION_MAJOR}::Widgets)
macro(qt_wrap_ui)
	if(QT_VERSION_MAJOR EQUAL 6)
		qt6_wrap_ui(${ARGN})
	elseif(QT_VERSION_MAJOR EQUAL 5)
		qt5_wrap_ui(${ARGN})
	endif()
	endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_subdirectory(visualization_tools)
add_subdirectory(motion_planning_tasks)

install(DIRECTORY icons DESTINATION share)

pluginlib_export_plugin_description_file(rviz_common motion_planning_tasks_rviz_plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(motion_planning_tasks_utils
	motion_planning_tasks_properties
	motion_planning_tasks_rviz_plugin
	moveit_task_visualization_tools
)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(Boost)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_visualization)
ament_export_dependencies(moveit_task_constructor_core)
ament_export_dependencies(moveit_task_constructor_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)
ament_export_dependencies(rviz_default_plugins)
ament_export_dependencies(rviz_ogre_vendor)
ament_package()
