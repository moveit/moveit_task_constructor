# pybind11 must use the ROS python version
set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION})
find_package(pybind11 3.0 REQUIRED)

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
