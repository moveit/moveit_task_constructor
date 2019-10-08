cmake_minimum_required(VERSION 2.8.3)
project(moveit_task_constructor_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/pick_place_task.cpp)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo src/moveit_task_constructor_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME moveit_task_constructor_demo)
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
