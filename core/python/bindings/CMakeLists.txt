set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/task_constructor)
add_library(${PROJECT_NAME}_python_tools SHARED
	${INCLUDES}/properties.h
	src/properties.cpp
)
target_link_libraries(${PROJECT_NAME}_python_tools PUBLIC ${PROJECT_NAME}
	pybind11::pybind11 py_binding_tools::py_binding_tools)
# Use minimum-size optimization for pybind11 bindings
target_link_libraries(${PROJECT_NAME}_python_tools PUBLIC pybind11::opt_size)

# moveit.task_constructor
pybind11_add_module(pymoveit_mtc
	src/solvers.cpp
	src/core.cpp
	src/stages.cpp
	src/module.cpp
)
target_include_directories(pymoveit_mtc PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(pymoveit_mtc PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_stages ${PROJECT_NAME}_python_tools)

# install libs
install(TARGETS ${PROJECT_NAME}_python_tools
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib)
install(TARGETS pymoveit_mtc
	LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
)
