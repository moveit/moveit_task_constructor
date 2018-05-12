find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(${PYTHON_INCLUDE_PATH})

add_library(_core SHARED
	conversions.cpp
	ros_init.cpp
	properties.cpp
	solvers.cpp
	core.cpp
)
target_link_libraries(_core ${PROJECT_NAME} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(_core PROPERTIES PREFIX "")

add_library(_stages SHARED
	stages.cpp
)
target_link_libraries(_stages _core ${PROJECT_NAME}_stages)
set_target_properties(_stages PROPERTIES PREFIX "")

install(TARGETS _core _stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
