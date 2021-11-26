cmake_minimum_required(VERSION 3.5)
project(rviz_marker_tools)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

set(HEADERS
	${PROJECT_INCLUDE}/marker_creation.h
)

add_library(${PROJECT_NAME} SHARED
	${HEADERS}
	src/marker_creation.cpp
)

ament_target_dependencies(${PROJECT_NAME} visualization_msgs geometry_msgs Eigen3)
target_include_directories(${PROJECT_NAME}
  PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz2)
ament_export_dependencies(tf2_eigen)
ament_package()
