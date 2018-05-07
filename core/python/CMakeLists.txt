find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(${PYTHON_INCLUDE_PATH})

add_library(_core SHARED
	core.cpp
	ros_init.cpp
	task.cpp
)

target_link_libraries(_core ${PROJECT_NAME} ${PROJECT_NAME}_stages ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(_core PROPERTIES PREFIX "")

install(TARGETS _core
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
