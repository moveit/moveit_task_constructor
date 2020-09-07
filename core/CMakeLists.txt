cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_constructor_core)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roslint
	eigen_conversions
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_msgs
	roscpp
	visualization_msgs
	rviz_marker_tools
)

catkin_python_setup()

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
		${PROJECT_NAME}_stages
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_task_constructor_msgs
		visualization_msgs
)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

# check for MOVEIT_MASTER
include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
CHECK_INCLUDE_FILE_CXX(moveit/collision_detection/collision_env.h MOVEIT_MASTER)
if(NOT MOVEIT_MASTER)
  set(MOVEIT_MASTER 0)
endif()
add_definitions(-DMOVEIT_MASTER=${MOVEIT_MASTER})

add_compile_options(-fvisibility-inlines-hidden)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/task_constructor)

add_subdirectory(src)
add_subdirectory(python)
add_subdirectory(test)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
	PATTERN "*_p.h" EXCLUDE)
install(FILES
	motion_planning_stages_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
