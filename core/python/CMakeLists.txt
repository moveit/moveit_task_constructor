# C++ wrapper code
add_compile_options(-Wno-deprecated-declarations)  # suppress warnings due to std::auto_ptr
add_subdirectory(wrapper)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	# Add folders to be run by python nosetests
	catkin_add_nosetests(test)

	# Add rostests
	add_rostest(test/rostest_mtc.test)
endif()

roslint_python()
