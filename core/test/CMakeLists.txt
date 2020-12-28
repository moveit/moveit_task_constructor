#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(ament_cmake_gmock REQUIRED)
	find_package(ros_testing REQUIRED)

	add_library(gtest_utils SHARED gtest_value_printers.cpp models.cpp)
	target_link_libraries(gtest_utils ${PROJECT_NAME})

	ament_add_gtest(${PROJECT_NAME}-test-stage test_stage.cpp)
	target_link_libraries(${PROJECT_NAME}-test-stage ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils)

	ament_add_gtest(${PROJECT_NAME}-test-container test_container.cpp)
	target_link_libraries(${PROJECT_NAME}-test-container ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils)

	ament_add_gtest(${PROJECT_NAME}-test-serial test_serial.cpp)
	target_link_libraries(${PROJECT_NAME}-test-serial ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils)

	ament_add_gtest(${PROJECT_NAME}-test-properties test_properties.cpp)
	target_link_libraries(${PROJECT_NAME}-test-properties ${PROJECT_NAME})

	ament_add_gmock(${PROJECT_NAME}-test-cost_queue test_cost_queue.cpp)
	target_link_libraries(${PROJECT_NAME}-test-cost_queue ${PROJECT_NAME})

	ament_add_gmock(${PROJECT_NAME}-test-interface_state test_interface_state.cpp)
	target_link_libraries(${PROJECT_NAME}-test-interface_state ${PROJECT_NAME} gtest_utils)

	ament_add_gtest(${PROJECT_NAME}-test-cost_terms test_cost_terms.cpp)
	target_link_libraries(${PROJECT_NAME}-test-cost_terms ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils)

	# building these integration tests works without moveit config packages
	ament_add_gtest(pick_ur5 pick_ur5.cpp)
	target_link_libraries(pick_ur5 ${PROJECT_NAME}_stages)

	ament_add_gtest(pick_pr2 pick_pr2.cpp)
	target_link_libraries(pick_pr2 ${PROJECT_NAME}_stages)

	ament_add_gtest(pick_pa10 pick_pa10.cpp)
	target_link_libraries(pick_pa10 ${PROJECT_NAME}_stages)

	# TODO(JafarAbdi): Port integration tests
	#	# running these integrations test naturally requires the moveit configs
	#	find_package(tams_ur5_setup_moveit_config QUIET)
	#	if(tams_ur5_setup_moveit_config_FOUND)
	#		add_rostest(pick_ur5.test)
	#	endif()
	#	find_package(pr2_moveit_config_FOUND QUIET)
	#	if(pr2_moveit_config_FOUND)
	#		add_rostest(pick_pr2.test)
	#	endif()
	#	find_package(pa10_shadow_moveit_config QUIET)
	#	if(pa10_shadow_moveit_config_FOUND)
	#		add_rostest(pick_pa10.test)
	#	endif()
endif()
