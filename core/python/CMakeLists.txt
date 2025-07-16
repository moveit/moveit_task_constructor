# We rely on pybind11's smart_holder branch imported pybind11 via git submodule

find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
# C++ wrapper code
add_subdirectory(bindings)

if(BUILD_TESTING AND NOT DEFINED ENV{PRELOAD})
	find_package(ament_cmake_pytest REQUIRED)
	find_package(launch_testing_ament_cmake)
	set(_pytest_tests test/test_mtc.py)
	foreach(_test_path ${_pytest_tests})
		get_filename_component(_test_name ${_test_path} NAME_WE)
		ament_add_pytest_test(${_test_name} ${_test_path}
			APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
			TIMEOUT 60
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	endforeach()

	# add_launch_test(test/test.launch.py TARGET rostest_mtc ARGS "exe:=${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_mtc.py")
	# add_launch_test(test/test.launch.py TARGET rostest_mps ARGS "exe:=${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_mps.py")
	# add_launch_test(test/test.launch.py TARGET rostest_trampoline ARGS "exe:=${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_trampoline.py")
endif()
