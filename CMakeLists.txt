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
	StageDescription.msg
	StageStatistics.msg
	TaskDescription.msg
	TaskStatistics.msg
	Solution.msg
	SubSolution.msg
	SubTrajectory.msg
)
add_service_files(DIRECTORY srv
	FILES
	GetSolution.srv
)
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	INCLUDE_DIRS include
)

set(CMAKE_CXX_STANDARD 14)
set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(src)
add_subdirectory(visualization)

install(DIRECTORY include/ DESTINATION include)
install(FILES
	motion_planning_tasks_rviz_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
