cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_msgs)

set(MSG_DEPS moveit_msgs std_msgs visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
	actionlib_msgs
	genmsg
	geometry_msgs
	message_generation
	${MSG_DEPS}
)

# ROS messages, services and actions
add_message_files(DIRECTORY msg FILES
	Property.msg
	Solution.msg
	SolutionInfo.msg
	StageDescription.msg
	StageStatistics.msg
	SubSolution.msg
	SubTrajectory.msg
	TaskDescription.msg
	TaskStatistics.msg
)

add_service_files(DIRECTORY srv FILES
	GetSolution.srv
)

add_action_files(DIRECTORY action FILES
	ExecuteTaskSolution.action
	SampleGraspPoses.action
)

generate_messages(DEPENDENCIES
								   actionlib_msgs
									 geometry_msgs
									 ${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS
	  actionlib_msgs
		geometry_msgs
		message_runtime
		${MSG_DEPS}
)
