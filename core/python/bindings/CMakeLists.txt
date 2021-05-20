# python tools support lib
set(TOOLS_LIB_NAME moveit_python_tools)
set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/python_tools)
add_library(${TOOLS_LIB_NAME} SHARED
	${INCLUDES}/ros_init.h
	${INCLUDES}/ros_types.h
	src/ros_init.cpp
	src/ros_types.cpp
)
target_include_directories(${TOOLS_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${TOOLS_LIB_NAME} PUBLIC pybind11::pybind11 ${Boost_LIBRARIES} ${roscpp_LIBRARIES})

install(TARGETS ${TOOLS_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

# moveit.python_tools
pybind11_add_module(pymoveit_python_tools src/python_tools.cpp)
target_link_libraries(pymoveit_python_tools PRIVATE ${TOOLS_LIB_NAME})
set_target_properties(pymoveit_python_tools PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# moveit.task_constructor
set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/task_constructor)
pybind11_add_module(pymoveit_mtc
	${INCLUDES}/properties.h

	src/properties.cpp
	src/solvers.cpp
	src/core.cpp
	src/stages.cpp
	src/module.cpp
)
target_include_directories(pymoveit_mtc PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(pymoveit_mtc PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_stages ${TOOLS_LIB_NAME})
set_target_properties(pymoveit_mtc PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# install python libs
install(TARGETS pymoveit_python_tools pymoveit_mtc
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
