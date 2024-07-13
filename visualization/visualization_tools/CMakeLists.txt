set(MOVEIT_LIB_NAME moveit_task_visualization_tools)

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
target_link_libraries(${MOVEIT_LIB_NAME}
	${QT_LIBRARIES}
	rviz_ogre_vendor::OgreMain
	fmt::fmt
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC include
)
ament_target_dependencies(${MOVEIT_LIB_NAME}
	Boost
	pluginlib
	moveit_task_constructor_msgs
	moveit_ros_visualization
	moveit_core
	rclcpp
	rviz_common
	rviz_default_plugins
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}
	EXPORT export_${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
