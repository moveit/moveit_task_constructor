set(MOVEIT_LIB_NAME motion_planning_tasks_rviz_plugin)

qt_wrap_ui(UIC_FILES
	task_panel.ui
	task_view.ui
	global_settings.ui
)

add_library(${MOVEIT_LIB_NAME} SHARED
	factory_model.cpp
	icons.cpp
	job_queue.cpp
	local_task_model.cpp
	meta_task_list_model.cpp
	pluginlib_factory.h
	plugin_init.cpp
	remote_task_model.cpp
	task_display.cpp
	task_list_model.cpp
	task_panel.cpp
	task_panel_p.h

	${UIC_FILES}

	resources.qrc
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
	motion_planning_tasks_utils motion_planning_tasks_properties moveit_task_visualization_tools
	${QT_LIBRARIES}
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
	PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	# https://stackoverflow.com/questions/47175683/cmake-target-link-libraries-propagation-of-include-directories
	PUBLIC $<TARGET_PROPERTY:motion_planning_tasks_utils,INTERFACE_INCLUDE_DIRECTORIES>
	PUBLIC $<TARGET_PROPERTY:motion_planning_tasks_properties,INTERFACE_INCLUDE_DIRECTORIES>
	PUBLIC $<TARGET_PROPERTY:moveit_task_visualization_tools,INTERFACE_INCLUDE_DIRECTORIES>
)

install(TARGETS ${MOVEIT_LIB_NAME}
	EXPORT export_${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
