# We rely on pybind11's smart_holder branch imported pybind11 via git submodule

find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

# Use minimum-size optimization for pybind11 bindings
add_compile_options("-Os")

# configure pybind11 install for use by downstream packages in install space
set(PYBIND11_INSTALL ON CACHE INTERNAL "Install pybind11")
set(CMAKE_INSTALL_INCLUDEDIR include/moveit/python)
set(PYBIND11_CMAKECONFIG_INSTALL_DIR share/${PROJECT_NAME}/cmake
    CACHE INTERNAL "install path for pybind11 cmake files")

# source pybind11 folder, which exposes its targets and installs them
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/CMakeLists.txt")
	message("Missing content of submodule pybind11: Use 'git clone --recurse-submodule' in future.\n"
	        "Checking out content automatically")
	execute_process(COMMAND git submodule init WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	execute_process(COMMAND git submodule update WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
add_subdirectory(pybind11)

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
