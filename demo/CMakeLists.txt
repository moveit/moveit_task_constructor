cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_constructor_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartesian src/cartesian.cpp)
target_link_libraries(cartesian ${catkin_LIBRARIES})

add_executable(modular src/modular.cpp)
target_link_libraries(modular ${catkin_LIBRARIES})

add_executable(pick_place_demo src/pick_place_demo.cpp src/pick_place_task.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES})

install(TARGETS cartesian modular pick_place_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
