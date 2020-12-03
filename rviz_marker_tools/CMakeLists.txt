cmake_minimum_required(VERSION 3.1.3)
project(rviz_marker_tools)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	visualization_msgs
	roscpp
	rviz
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		visualization_msgs
		roscpp
		rviz
)

set(CMAKE_CXX_STANDARD 14)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

set(HEADERS
	${PROJECT_INCLUDE}/marker_creation.h
)

add_library(${PROJECT_NAME}
	${HEADERS}
	src/marker_creation.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}
  PUBLIC include
  PRIVATE ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
