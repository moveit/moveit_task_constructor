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
	target_link_libraries(${PROJECT_NAME}-test-container ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-properties test_properties.cpp)
	target_link_libraries(${PROJECT_NAME}-test-properties ${PROJECT_NAME} gtest_main)

	catkin_add_gmock(${PROJECT_NAME}-test-cost_queue test_cost_queue.cpp)
	target_link_libraries(${PROJECT_NAME}-test-cost_queue ${PROJECT_NAME} gtest_main)

	catkin_add_gtest(${PROJECT_NAME}-test-interface_state test_interface_state.cpp)
	target_link_libraries(${PROJECT_NAME}-test-interface_state ${PROJECT_NAME} gtest_main)


	# building these integration tests works without moveit config packages
	add_executable(pick_ur5 pick_ur5.cpp)
	target_link_libraries(pick_ur5 ${PROJECT_NAME}_stages gtest)

	add_executable(pick_pr2 pick_pr2.cpp)
	target_link_libraries(pick_pr2 ${PROJECT_NAME}_stages gtest)

	add_executable(pick_pa10 pick_pa10.cpp)
	target_link_libraries(pick_pa10 ${PROJECT_NAME}_stages gtest)

	# running these integrations test naturally requires the moveit configs
	find_package(tams_ur5_setup_moveit_config QUIET)
	if(tams_ur5_setup_moveit_config_FOUND)
		add_rostest(pick_ur5.test)
	endif()
	find_package(pr2_moveit_config_FOUND QUIET)
	if(pr2_moveit_config_FOUND)
		add_rostest(pick_pr2.test)
	endif()
	find_package(pa10_shadow_moveit_config QUIET)
	if(pa10_shadow_moveit_config_FOUND)
		add_rostest(pick_pa10.test)
	endif()
endif()
