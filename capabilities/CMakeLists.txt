cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_capabilities)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_move_group
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  pluginlib
  std_msgs
)

moveit_build_options()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    moveit_task_constructor_msgs
    std_msgs
)

# Check for availability of moveit::core::MoveItErrorCode::toString
set(CMAKE_REQUIRED_INCLUDES ${catkin_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${catkin_LIBRARIES})
set(CMAKE_REQUIRED_FLAGS -Wno-error)
include(CheckCXXSymbolExists)
check_cxx_symbol_exists(moveit::core::MoveItErrorCode::toString "moveit/utils/moveit_error_code.h" HAVE_MOVEIT_ERROR_CODE_TO_STRING)
if(NOT HAVE_MOVEIT_ERROR_CODE_TO_STRING)
  set(HAVE_MOVEIT_ERROR_CODE_TO_STRING 0)
endif()

add_library(${PROJECT_NAME}
   src/execute_task_solution_capability.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_MOVEIT_ERROR_CODE_TO_STRING=${HAVE_MOVEIT_ERROR_CODE_TO_STRING})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
