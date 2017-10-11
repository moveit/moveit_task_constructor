cmake_minimum_required(VERSION 2.6.12)

project(moveit_task_constructor)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
	eigen_conversions
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)

add_library(${PROJECT_NAME}
	src/storage.cpp
	src/subtask.cpp
	src/container.cpp
	src/task.cpp
	src/debug.cpp

	src/subtask_p.h
	src/container_p.h
	include/${PROJECT_NAME}/utils.h
	include/${PROJECT_NAME}/storage.h
	include/${PROJECT_NAME}/subtask.h
	include/${PROJECT_NAME}/container.h
	include/${PROJECT_NAME}/task.h
	include/${PROJECT_NAME}/debug.h
)

add_subdirectory(src/subtasks)
add_subdirectory(src/demo)
add_subdirectory(src/test)
