set(MOVEIT_LIB_NAME moveit_task_panel_rviz_plugin)

qt_wrap_ui(UIC_FILES task_panel.ui)

add_library(${MOVEIT_LIB_NAME}
	mainloop_processing.cpp
	stage_wrapper.cpp
	task_model.cpp
	task_panel.cpp
	${UIC_FILES}
)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS ${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
