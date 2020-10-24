#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	catkin_add_gtest(${PROJECT_NAME}-test-merge-models test_merge_models.cpp)
	target_link_libraries(${PROJECT_NAME}-test-merge-models
		motion_planning_tasks_utils gtest_main)

	catkin_add_gmock(${PROJECT_NAME}-test-solution-models test_solution_models.cpp)
	target_link_libraries(${PROJECT_NAME}-test-solution-models
		motion_planning_tasks_rviz_plugin gtest_main)

	add_rostest_gtest(${PROJECT_NAME}-test-task_model test_task_model.launch test_task_model.cpp)
	target_link_libraries(${PROJECT_NAME}-test-task_model
		motion_planning_tasks_rviz_plugin ${catkin_LIBRARIES} gtest)
endif()
