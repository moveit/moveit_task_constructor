cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_core)

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_marker_tools REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

# check for MOVEIT_MASTER
include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
CHECK_INCLUDE_FILE_CXX(moveit/collision_detection/collision_env.h MOVEIT_MASTER)
if(NOT MOVEIT_MASTER)
  set(MOVEIT_MASTER 0)
endif()
add_definitions(-DMOVEIT_MASTER=${MOVEIT_MASTER})

add_compile_options(-fvisibility-inlines-hidden)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/task_constructor)

add_subdirectory(src)
# add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include
	PATTERN "*_p.h" EXCLUDE)
install(FILES
	motion_planning_stages_plugin_description.xml
        DESTINATION share)

ament_package()
