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

add_compile_options(-std=c++14)

add_library(${PROJECT_NAME}_subtasks
	src/subtasks/move.cpp
	src/subtasks/current_state.cpp
	src/subtasks/gripper.cpp
	src/subtasks/generate_grasp_pose.cpp
	src/subtasks/cartesian_position_motion.cpp
)
target_link_libraries(${PROJECT_NAME}_subtasks ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
	src/subtask.cpp
	src/task.cpp
	src/debug.cpp
)

add_subdirectory(src/demo)

add_subdirectory(src/test)
