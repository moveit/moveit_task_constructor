cmake_minimum_required(VERSION 2.8.12)
project(moveit_task_constructor_msgs)

set(MSG_DEPS moveit_msgs visualization_msgs)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

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

catkin_package(DEPENDS ${MSG_DEPS})
