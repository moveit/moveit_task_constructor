#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	add_library(gtest_utils gtest_value_printers.cpp models.cpp stage_mockups.cpp)
	target_link_libraries(gtest_utils ${PROJECT_NAME})

	macro(mtc_add_test TYPE)
		# Split ARGN into .test file and other sources
		set(TEST_FILE ${ARGN})
		set(SOURCES ${ARGN})
		list(FILTER TEST_FILE INCLUDE REGEX "\.test$")
		list(FILTER SOURCES EXCLUDE REGEX "\.test$")
		# Determine TARGET name from first source file
		list(GET SOURCES 0 TEST_NAME)
		string(REGEX REPLACE "\.cpp$" "" TEST_NAME ${TEST_NAME})
		string(REGEX REPLACE "_" "-" TEST_NAME ${TEST_NAME})
		# Configure build target
		if(TEST_FILE) # Add rostest if .test file was specified
			_add_rostest_google_test(${TYPE} ${PROJECT_NAME}-${TEST_NAME} ${TEST_FILE} ${SOURCES})
		else()
			_catkin_add_google_test(${TYPE} ${PROJECT_NAME}-${TEST_NAME} ${SOURCES})
		endif()
		target_link_libraries(${PROJECT_NAME}-${TEST_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_stages gtest_utils gtest_main)
	endmacro()
	macro(mtc_add_gtest)
		mtc_add_test("gtest" ${ARGN})
	endmacro()
	macro(mtc_add_gmock)
		mtc_add_test("gmock" ${ARGN})
	endmacro()

	mtc_add_gmock(test_mockups.cpp)
	mtc_add_gtest(test_stage.cpp)
	mtc_add_gtest(test_container.cpp)
	mtc_add_gmock(test_serial.cpp)
	mtc_add_gmock(test_pruning.cpp)
	mtc_add_gtest(test_properties.cpp)
	mtc_add_gtest(test_cost_terms.cpp)

	mtc_add_gmock(test_fallback.cpp)
	mtc_add_gmock(test_cost_queue.cpp)
	mtc_add_gmock(test_interface_state.cpp)

	mtc_add_gtest(test_move_to.cpp move_to.test)
	mtc_add_gtest(test_move_relative.cpp move_relative.test)

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
