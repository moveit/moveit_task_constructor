cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_msgs)

set(MSG_DEPS moveit_msgs visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
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
	TrajectoryExecutionInfo.msg
)

add_service_files(DIRECTORY srv FILES
	GetSolution.srv
)

add_action_files(DIRECTORY action FILES
	ExecuteTaskSolution.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
