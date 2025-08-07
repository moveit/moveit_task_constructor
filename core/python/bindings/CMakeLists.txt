set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/task_constructor)
add_library(${PROJECT_NAME}_python_tools SHARED
	${INCLUDES}/properties.h
	src/properties.cpp
)
target_link_libraries(${PROJECT_NAME}_python_tools PUBLIC ${PROJECT_NAME} pybind11::pybind11)
# Use minimum-size optimization for pybind11 bindings
target_link_libraries(${PROJECT_NAME}_python_tools PUBLIC pybind11::opt_size)

# catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

# moveit.task_constructor
pybind11_add_module(pymoveit_mtc
	src/solvers.cpp
	src/core.cpp
	src/stages.cpp
	src/module.cpp
)
target_link_libraries(pymoveit_mtc PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_stages ${PROJECT_NAME}_python_tools)
set_target_properties(pymoveit_mtc PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# install libs
install(TARGETS ${PROJECT_NAME}_python_tools
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS pymoveit_mtc
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
