#############
## Testing ##
#############

## Add gtest based cpp test target
if (BUILD_TESTING AND NOT (CMAKE_CXX_FLAGS MATCHES "-fsanitize"))
	find_package(ament_cmake_gtest REQUIRED)
	find_package(launch_testing_ament_cmake REQUIRED)
	find_package(moveit_task_constructor_core REQUIRED)

	ament_add_gtest_executable(test_execution test_task_execution.cpp)
	add_launch_test(test_execution.launch.py TARGET test_execution
	                ARGS "test_binary:=$<TARGET_FILE:test_execution>")
	target_link_libraries(test_execution ${moveit_task_constructor_core_TARGETS})
endif()
