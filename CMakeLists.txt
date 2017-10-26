cmake_minimum_required(VERSION 2.6.12)
project(moveit_task_constructor)

set(MSG_DEPS moveit_msgs visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	genmsg ${MSG_DEPS}
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
	moveit_ros_visualization
	eigen_conversions
	rviz
)

# ROS messages, services and actions
add_message_files(DIRECTORY msg
	FILES
	Stage.msg
	Task.msg
	SubSolution.msg
	SubTrajectory.msg
	Solution.msg
)
add_service_files(DIRECTORY srv
	FILES
	GetInterfaceState.srv
	GetSolution.srv
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
	src/introspection.cpp

	src/stage_p.h
	src/container_p.h
	include/${PROJECT_NAME}/utils.h
	include/${PROJECT_NAME}/storage.h
	include/${PROJECT_NAME}/stage.h
	include/${PROJECT_NAME}/container.h
	include/${PROJECT_NAME}/task.h
	include/${PROJECT_NAME}/introspection.h
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES
	task_panel_rviz_plugin_description.xml
	task_solution_rviz_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_subdirectory(src/stages)
add_subdirectory(src/demo)
add_subdirectory(src/test)

add_subdirectory(visualization)
