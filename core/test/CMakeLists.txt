#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	add_library(gtest_utils gtest_value_printers.cpp models.cpp)
	target_link_libraries(gtest_utils ${PROJECT_NAME})

	catkin_add_gtest(${PROJECT_NAME}-test-stage test_stage.cpp)
	target_link_libraries(${PROJECT_NAME}-test-stage ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-container test_container.cpp)
	target_link_libraries(${PROJECT_NAME}-test-container ${PROJECT_NAME} gtest_utils gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-properties test_properties.cpp)
	target_link_libraries(${PROJECT_NAME}-test-properties ${PROJECT_NAME} gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-cost_queue test_cost_queue.cpp)
	target_link_libraries(${PROJECT_NAME}-test-cost_queue ${PROJECT_NAME} gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-interface_state test_interface_state.cpp)
	target_link_libraries(${PROJECT_NAME}-test-interface_state ${PROJECT_NAME} gtest_main)
endif()
