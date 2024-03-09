cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS messages, services and actions
set(msg_files
	msg/Property.msg
	msg/Solution.msg
	msg/SolutionInfo.msg
	msg/StageDescription.msg
	msg/StageStatistics.msg
	msg/SubSolution.msg
	msg/SubTrajectory.msg
	msg/TaskDescription.msg
	msg/TaskStatistics.msg
	msg/TrajectoryExecutionInfo.msg
)

set(srv_files
	srv/GetSolution.srv
)

set(action_files
	action/ExecuteTaskSolution.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	${action_files}
	DEPENDENCIES
	builtin_interfaces
	moveit_msgs
	visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
