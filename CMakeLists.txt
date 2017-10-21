cmake_minimum_required(VERSION 2.6.12)

project(moveit_task_constructor)

set(MSG_DEPS trajectory_msgs moveit_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	genmsg ${MSG_DEPS}
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
	eigen_conversions
)

# ROS messages, services and actions
add_message_files(DIRECTORY msg
	FILES
	Stage.msg
	Task.msg
	Solution.msg
)
add_service_files(DIRECTORY srv
	FILES
	GetInterfaceState.srv
	GetSolutionTrajectory.srv
)
generate_messages(DEPENDENCIES ${MSG_DEPS})

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
	src/stage.cpp
	src/container.cpp
	src/task.cpp
	src/debug.cpp

	src/stage_p.h
	src/container_p.h
	include/${PROJECT_NAME}/utils.h
	include/${PROJECT_NAME}/storage.h
	include/${PROJECT_NAME}/stage.h
	include/${PROJECT_NAME}/container.h
	include/${PROJECT_NAME}/task.h
	include/${PROJECT_NAME}/debug.h
)

add_subdirectory(src/stages)
add_subdirectory(src/demo)
add_subdirectory(src/test)
