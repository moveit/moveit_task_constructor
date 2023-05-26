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
target_link_libraries(pymoveit_mtc PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_stages py_binding_tools::py_binding_tools)

# install python libs
install(TARGETS pymoveit_mtc
	LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
)
