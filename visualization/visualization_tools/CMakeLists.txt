set(MOVEIT_LIB_NAME moveit_task_visualization_tools)
find_package(moveit_core REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)
set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/visualization_tools)

set(HEADERS
	${PROJECT_INCLUDE}/display_solution.h
	${PROJECT_INCLUDE}/marker_visualization.h
	${PROJECT_INCLUDE}/task_solution_panel.h
	${PROJECT_INCLUDE}/task_solution_visualization.h
)

add_library(${MOVEIT_LIB_NAME} SHARED
	${HEADERS}

	src/display_solution.cpp
	src/marker_visualization.cpp
	src/task_solution_panel.cpp
	src/task_solution_visualization.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
    target_link_libraries(moveit_task_visualization_tools PUBLIC
      ${moveit_task_constructor_msgs_TARGETS}
      moveit_core::moveit_collision_detection
      moveit_core::moveit_collision_detection_bullet
      moveit_core::moveit_collision_detection_fcl
      moveit_core::moveit_collision_distance_field
      moveit_core::moveit_constraint_samplers
      moveit_core::moveit_distance_field
      moveit_core::moveit_dynamics_solver
      moveit_core::moveit_exceptions
      moveit_core::moveit_kinematic_constraints
      moveit_core::moveit_kinematics_base
      moveit_core::moveit_kinematics_metrics
      moveit_core::moveit_macros
      moveit_core::moveit_planning_interface
      moveit_core::moveit_planning_scene
      moveit_core::moveit_robot_model
      moveit_core::moveit_robot_state
      moveit_core::moveit_robot_trajectory
      moveit_core::moveit_smoothing_base
      moveit_core::moveit_test_utils
      moveit_core::moveit_trajectory_processing
      moveit_core::moveit_transforms
      moveit_core::moveit_utils
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin_core
      moveit_ros_visualization::moveit_robot_state_rviz_plugin
      moveit_ros_visualization::moveit_robot_state_rviz_plugin_core
      moveit_ros_visualization::moveit_rviz_plugin_render_tools
      moveit_ros_visualization::moveit_trajectory_rviz_plugin
      moveit_ros_visualization::moveit_trajectory_rviz_plugin_core
      pluginlib::pluginlib
      rclcpp::rclcpp
      rviz_common::rviz_common
      rviz_default_plugins::rviz_default_plugins
    )

target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC include
)


install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}
	EXPORT export_${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
