add_library(${PROJECT_NAME}_stages SHARED
	${PROJECT_INCLUDE}/stages/modify_planning_scene.h
	${PROJECT_INCLUDE}/stages/fix_collision_objects.h

	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/fixed_state.h
	${PROJECT_INCLUDE}/stages/fixed_cartesian_poses.h
	${PROJECT_INCLUDE}/stages/generate_pose.h
	${PROJECT_INCLUDE}/stages/generate_random_pose.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/generate_place_pose.h
	${PROJECT_INCLUDE}/stages/compute_ik.h
	${PROJECT_INCLUDE}/stages/passthrough.h
	${PROJECT_INCLUDE}/stages/noop.h
	${PROJECT_INCLUDE}/stages/predicate_filter.h

	${PROJECT_INCLUDE}/stages/connect.h
	${PROJECT_INCLUDE}/stages/move_to.h
	${PROJECT_INCLUDE}/stages/move_relative.h

	${PROJECT_INCLUDE}/stages/simple_grasp.h
	${PROJECT_INCLUDE}/stages/pick.h

	modify_planning_scene.cpp
	fix_collision_objects.cpp

	current_state.cpp
	fixed_state.cpp
	fixed_cartesian_poses.cpp
	generate_pose.cpp
	generate_random_pose.cpp
	generate_grasp_pose.cpp
	generate_place_pose.cpp
	compute_ik.cpp
	passthrough.cpp
	predicate_filter.cpp

	connect.cpp
	move_to.cpp
	move_relative.cpp

	simple_grasp.cpp
	pick.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_stages
	moveit_core
	geometry_msgs
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_msgs
	rclcpp
	tf2_eigen
	visualization_msgs
)

add_library(${PROJECT_NAME}_stage_plugins SHARED
	plugins.cpp
)
target_link_libraries(${PROJECT_NAME}_stage_plugins ${PROJECT_NAME}_stages)

install(TARGETS ${PROJECT_NAME}_stages ${PROJECT_NAME}_stage_plugins
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
