cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_core)

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roslint
	tf2_eigen
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_msgs
	roscpp
	visualization_msgs
	rviz_marker_tools
	py_binding_tools
)

option(MOVEIT_CI_WARNINGS "Enable all warnings used by CI" OFF) # We use our own set of warnings
moveit_build_options()

catkin_python_setup()

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
		${PROJECT_NAME}_stages
		${PROJECT_NAME}_stage_plugins
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_task_constructor_msgs
		rviz_marker_tools
		visualization_msgs
	CFG_EXTRAS pybind11.cmake
)

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
