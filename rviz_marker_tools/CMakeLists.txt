cmake_minimum_required(VERSION 3.5)
project(rviz_marker_tools)

find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
	eigen3_cmake_module
	Eigen3
	geometry_msgs
	rclcpp
	tf2_eigen
	urdfdom_headers
	std_msgs
	visualization_msgs
)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
	find_package(${dependency} REQUIRED)
endforeach()

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

add_library(${PROJECT_NAME} SHARED
	src/marker_creation.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
