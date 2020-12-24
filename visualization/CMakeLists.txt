cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_visualization)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
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

# Qt Stuff
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
macro(qt_wrap_ui)
	qt5_wrap_ui(${ARGN})
endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(visualization_tools)
# add_subdirectory(motion_planning_tasks)

install(FILES
	motion_planning_tasks_rviz_plugin_description.xml
	DESTINATION include)

install(DIRECTORY icons DESTINATION share)
