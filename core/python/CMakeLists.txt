# We rely on pybind11's smart_holder branch imported pybind11 via git submodule

# pybind11 must use the ROS python version
set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION_STRING})

# Use minimum-size optimization for pybind11 bindings
add_compile_options("-Os")

# create symlink to grant access to downstream packages in devel space
add_custom_target(pybind11_devel_symlink ALL COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_SOURCE_DIR}/pybind11
	${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/pybind11)

# configure pybind11 install for use by downstream packages in install space
set(PYBIND11_INSTALL ON CACHE INTERNAL "Install pybind11")
set(CMAKE_INSTALL_INCLUDEDIR include/moveit/python)
set(PYBIND11_CMAKECONFIG_INSTALL_DIR ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
    CACHE INTERNAL "install path for pybind11 cmake files")

# source pybind11 folder, which exposes its targets and installs them
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/CMakeLists.txt")
	message("Missing content of submodule pybind11: Use 'git clone --recurse-submodule' in future.\n"
	        "Checking out content automatically")
	execute_process(COMMAND git submodule init WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	execute_process(COMMAND git submodule update WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
#catkin_lint: ignore_once subproject duplicate_cmd
add_subdirectory(pybind11)

# C++ wrapper code
add_subdirectory(bindings)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	# Add folders to be run by python nosetests
	if(DEFINED ENV{PRELOAD})
		message(WARNING "Skipping python tests due to asan")
	else()
		catkin_add_nosetests(test)

		# Add rostests
		add_rostest(test/rostest_mtc.test)
	endif()
endif()

roslint_python()
