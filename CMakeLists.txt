cmake_minimum_required(VERSION 2.6.12)

project(moveit_task_constructor)

find_package(catkin REQUIRED)

find_package(moveit_core REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include
	${moveit_core_INCLUDE_DIRS}
)

add_compile_options(-std=c++14)

add_library(${PROJECT_NAME}_subtasks
	src/subtasks/move.cpp
	src/subtasks/current_state.cpp
)

add_library(${PROJECT_NAME}
	src/subtask.cpp
	src/task.cpp
)

add_executable(test_plan_current_state src/test/test_plan_current_state.cpp)
target_link_libraries(test_plan_current_state ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(plan_pick src/plan_pick.cpp)
target_link_libraries(plan_pick ${PROJECT_NAME}_subtasks ${PROJECT_NAME})
