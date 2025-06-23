set(MOVEIT_LIB_NAME motion_planning_tasks_utils)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)
set(SOURCES
	flat_merge_proxy_model.cpp
	tree_merge_proxy_model.cpp
	icon.cpp
)
add_library(${MOVEIT_LIB_NAME} SHARED ${SOURCES})

target_link_libraries(${MOVEIT_LIB_NAME}
		rclcpp::rclcpp
		Qt${QT_VERSION_MAJOR}::Widgets
		Qt${QT_VERSION_MAJOR}::Core
)
target_include_directories(${MOVEIT_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

install(TARGETS ${MOVEIT_LIB_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
