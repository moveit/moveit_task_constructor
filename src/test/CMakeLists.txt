add_executable(test_plan_current_state test_plan_current_state.cpp)
target_link_libraries(test_plan_current_state ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_gripper test_plan_gripper.cpp)
target_link_libraries(test_plan_gripper ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_generate_grasp_pose test_plan_generate_grasp_pose.cpp)
target_link_libraries(test_plan_generate_grasp_pose ${PROJECT_NAME}_subtasks ${PROJECT_NAME})

add_executable(test_plan_cartesian_forward test_plan_cartesian_forward.cpp)
target_link_libraries(test_plan_cartesian_forward ${PROJECT_NAME}_subtasks ${PROJECT_NAME})
