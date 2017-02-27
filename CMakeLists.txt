cmake_minimum_required(VERSION 2.6.12)

project(moveit_task_constructor)

find_package(catkin REQUIRED)

find_package(moveit_core REQUIRED)

catkin_package(
)

add_executable(test_plan_pick src/test_plan_pick.cpp)
