set(MOVEIT_LIB_NAME motion_planning_tasks_utils)

set(SOURCES
	flat_merge_proxy_model.cpp
	tree_merge_proxy_model.cpp
	icon.cpp
)
add_library(${MOVEIT_LIB_NAME} SHARED ${SOURCES})

target_link_libraries(${MOVEIT_LIB_NAME}
	${QT_LIBRARIES}
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

install(TARGETS ${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
