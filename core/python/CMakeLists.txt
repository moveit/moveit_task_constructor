# C++ wrapper code
add_subdirectory(wrapper)

if(CATKIN_ENABLE_TESTING)
	add_subdirectory(test)
	# Add folders to be run by python nosetests
	catkin_add_nosetests(test)
endif()

roslint_python()
