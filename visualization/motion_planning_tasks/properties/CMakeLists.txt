set(MOVEIT_LIB_NAME motion_planning_tasks_properties)

set(SOURCES
	property_factory.cpp
	property_from_yaml.cpp
)

find_package(libyaml_vendor REQUIRED)
find_package(yaml REQUIRED)

add_library(${MOVEIT_LIB_NAME} SHARED ${SOURCES})

    target_link_libraries(motion_planning_tasks_properties PUBLIC
      moveit_task_constructor_core::moveit_task_constructor_core
      moveit_task_constructor_core::moveit_task_constructor_core_stage_plugins
      moveit_task_constructor_core::moveit_task_constructor_core_stages
      rviz_common::rviz_common
    )

target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
	PRIVATE ${YAML_INCLUDE_DIRS}
)


install(TARGETS ${MOVEIT_LIB_NAME}
	EXPORT export_${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
