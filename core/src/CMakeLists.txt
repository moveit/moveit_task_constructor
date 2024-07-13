add_library(${PROJECT_NAME} SHARED
	${PROJECT_INCLUDE}/container.h
	${PROJECT_INCLUDE}/container_p.h
	${PROJECT_INCLUDE}/cost_queue.h
	${PROJECT_INCLUDE}/introspection.h
	${PROJECT_INCLUDE}/marker_tools.h
	${PROJECT_INCLUDE}/merge.h
	${PROJECT_INCLUDE}/moveit_compat.h
	${PROJECT_INCLUDE}/properties.h
	${PROJECT_INCLUDE}/stage.h
	${PROJECT_INCLUDE}/stage_p.h
	${PROJECT_INCLUDE}/storage.h
	${PROJECT_INCLUDE}/task.h
	${PROJECT_INCLUDE}/task_p.h
	${PROJECT_INCLUDE}/utils.h

	${PROJECT_INCLUDE}/solvers/planner_interface.h
	${PROJECT_INCLUDE}/solvers/cartesian_path.h
	${PROJECT_INCLUDE}/solvers/joint_interpolation.h
	${PROJECT_INCLUDE}/solvers/pipeline_planner.h
	${PROJECT_INCLUDE}/solvers/multi_planner.h

	container.cpp
	cost_terms.cpp
	introspection.cpp
	marker_tools.cpp
	merge.cpp
	properties.cpp
	stage.cpp
	storage.cpp
	task.cpp
	utils.cpp

	solvers/planner_interface.cpp
	solvers/cartesian_path.cpp
	solvers/joint_interpolation.cpp
	solvers/pipeline_planner.cpp
	solvers/multi_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME}
	fmt
	moveit_core
	geometry_msgs
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_msgs
	rclcpp
	rviz_marker_tools
	visualization_msgs
)
target_include_directories(${PROJECT_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	       $<INSTALL_INTERFACE:include>
)

add_subdirectory(stages)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
