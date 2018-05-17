# C++ wrapper code
add_subdirectory(wrapper)

if(CATKIN_ENABLE_TESTING)
	find_package(moveit_resources)
	find_package(rostest)

	# Add folders to be run by python nosetests
	catkin_add_nosetests(test)

	# Add rostests
	add_rostest(test/rostest_mtc.test)
endif()

roslint_python()
