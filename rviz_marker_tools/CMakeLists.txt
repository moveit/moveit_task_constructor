cmake_minimum_required(VERSION 3.16)
project(rviz_marker_tools)

find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
	find_package(${dependency} REQUIRED)
endforeach()

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

add_library(${PROJECT_NAME} SHARED
	src/marker_creation.cpp
)

target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen  tf2_eigen::tf2_eigen tf2_geometry_msgs::tf2_geometry_msgs  urdfdom_headers::urdfdom_headers ${visualization_msgs_TARGETS})
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
