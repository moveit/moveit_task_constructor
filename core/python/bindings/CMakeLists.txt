# catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

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
install(TARGETS pymoveit_mtc
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
