set(MOVEIT_LIB_NAME motion_planning_tasks_rviz_plugin)

qt_wrap_ui(UIC_FILES task_panel.ui)

add_library(${MOVEIT_LIB_NAME}
	job_queue.cpp
	stage_wrapper.cpp
	task_model.cpp
	composite_proxy_model.cpp
	task_model_cache.cpp
	task_panel.cpp
	task_panel_p.h
	task_display.cpp
	plugin_init.cpp
	${UIC_FILES}
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME}
	moveit_task_visualization_tools
	${catkin_LIBRARIES} ${QT_LIBRARIES}
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

install(TARGETS ${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
