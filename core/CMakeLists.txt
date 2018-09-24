cmake_minimum_required(VERSION 2.8.12)
project(moveit_task_constructor_core)

find_package(catkin REQUIRED COMPONENTS
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

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/task_constructor)

add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES
	motion_planning_stages_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
