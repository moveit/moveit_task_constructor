cmake_minimum_required(VERSION 3.1)
project(moveit_task_constructor_core)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_msgs
	eigen_conversions
)

catkin_package(
	LIBRARIES
		moveit_task_constructor_core
		moveit_task_constructor_core_stages
	INCLUDE_DIRS
		include
)

set(CMAKE_CXX_STANDARD 14)
set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/task_constructor)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(demo)
add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include)
install(FILES
	motion_planning_stages_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
