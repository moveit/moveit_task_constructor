set(MOVEIT_LIB_NAME motion_planning_tasks_properties)

set(SOURCES
	property_factory.cpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML yaml-0.1)
if (YAML_FOUND)
	# Only cmake > 3.12 provides XXX_LINK_LIBRARIES. Find the absolute path manually
	find_library(YAML_LIBRARIES ${YAML_LIBRARIES} PATHS ${YAML_LIBRARY_DIRS})
	list(APPEND SOURCES property_from_yaml.cpp)
	add_definitions(-DHAVE_YAML)
endif()

add_library(${MOVEIT_LIB_NAME} SHARED ${SOURCES})

target_link_libraries(${MOVEIT_LIB_NAME}
	${QT_LIBRARIES} ${YAML_LIBRARIES}
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
	PRIVATE ${YAML_INCLUDE_DIRS}
)
ament_target_dependencies(${MOVEIT_LIB_NAME}
	moveit_task_constructor_core
	rviz_common
)

install(TARGETS ${MOVEIT_LIB_NAME}
	EXPORT export_${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
