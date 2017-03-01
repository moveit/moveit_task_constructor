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
)
target_link_libraries(${PROJECT_NAME}_subtasks ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
	src/subtask.cpp
	src/task.cpp
)

add_executable(plan_pick src/plan_pick.cpp)
target_link_libraries(plan_pick ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_current_state src/test/test_plan_current_state.cpp)
target_link_libraries(test_plan_current_state ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_gripper src/test/test_plan_gripper.cpp)
target_link_libraries(test_plan_gripper ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_generate_grasp_pose src/test/test_plan_generate_grasp_pose.cpp)
target_link_libraries(test_plan_generate_grasp_pose ${PROJECT_NAME}_subtasks ${PROJECT_NAME})
