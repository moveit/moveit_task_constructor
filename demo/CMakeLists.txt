cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_demo)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosparam_shortcuts REQUIRED)

include_directories(include)

add_executable(cartesian src/cartesian.cpp)
ament_target_dependencies(cartesian
                          rclcpp
                          moveit_core
                          moveit_task_constructor_core
)

add_executable(modular src/modular.cpp)
ament_target_dependencies(modular
                          rclcpp
                          moveit_core
                          moveit_task_constructor_core
)

add_executable(pick_place_demo src/pick_place_demo.cpp src/pick_place_task.cpp)
ament_target_dependencies(pick_place_demo
                          rclcpp
                          moveit_core
                          moveit_task_constructor_core
                          moveit_task_constructor_msgs
                          moveit_ros_planning_interface
                          rclcpp_action
                          tf2_eigen
                          rosparam_shortcuts
)

install(TARGETS cartesian modular pick_place_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()