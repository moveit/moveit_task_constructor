set(MOVEIT_LIB_NAME moveit_task_visualization_tools)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/visualization_tools)

set(HEADERS
	${PROJECT_INCLUDE}/display_solution.h
	${PROJECT_INCLUDE}/marker_visualization.h
	${PROJECT_INCLUDE}/task_solution_panel.h
	${PROJECT_INCLUDE}/task_solution_visualization.h
)

add_library(${MOVEIT_LIB_NAME}
	${HEADERS}

	src/display_solution.cpp
	src/marker_visualization.cpp
	src/task_solution_panel.cpp
	src/task_solution_visualization.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
	${catkin_LIBRARIES}
	${rviz_DEFAULT_PLUGIN_LIBRARIES}
	${OGRE_LIBRARIES}
	${QT_LIBRARIES}
	${Boost_LIBRARIES}
	fmt::fmt
)
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC include)
target_include_directories(${MOVEIT_LIB_NAME} SYSTEM
	PRIVATE ${catkin_INCLUDE_DIRS}
	PUBLIC ${rviz_OGRE_INCLUDE_DIRS}
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
