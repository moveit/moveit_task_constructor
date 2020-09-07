cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_constructor_capabilities)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_move_group
  moveit_task_constructor_msgs
  pluginlib
  std_msgs
)

catkin_package(
  LIBRARIES $PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_task_constructor_msgs
    std_msgs
)

# check for MOVEIT_MASTER
include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
CHECK_INCLUDE_FILE_CXX(moveit/collision_detection/collision_env.h MOVEIT_MASTER)
if(NOT MOVEIT_MASTER)
  set(MOVEIT_MASTER 0)
endif()
add_definitions(-DMOVEIT_MASTER=${MOVEIT_MASTER})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/execute_task_solution_capability.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
