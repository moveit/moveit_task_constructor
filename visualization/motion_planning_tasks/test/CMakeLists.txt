#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(ament_cmake_gmock REQUIRED)
	find_package(launch_testing_ament_cmake REQUIRED)

	ament_add_gtest(${PROJECT_NAME}-test-merge-models test_merge_models.cpp)
	target_link_libraries(${PROJECT_NAME}-test-merge-models
		motion_planning_tasks_utils)

	ament_add_gmock(${PROJECT_NAME}-test-solution-models test_solution_models.cpp)
	target_link_libraries(${PROJECT_NAME}-test-solution-models
		motion_planning_tasks_rviz_plugin)

	ament_add_gtest_executable(${PROJECT_NAME}-test-task_model test_task_model.cpp)
	target_link_libraries(${PROJECT_NAME}-test-task_model
		motion_planning_tasks_rviz_plugin)
	add_launch_test(test.launch.py
		ARGS "test_binary:=$<TARGET_FILE:${PROJECT_NAME}-test-task_model>")
endif()
