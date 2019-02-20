find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
include_directories(${PYTHON_INCLUDE_PATH})

# python tools support lib
set(TOOLS_LIB_NAME moveit_python_tools)
set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/python_tools)
add_library(${TOOLS_LIB_NAME} SHARED
	${INCLUDES}/conversions.h
	${INCLUDES}/ros_init.h

	src/conversions.cpp
	src/ros_init.cpp
	src/ros_types.cpp
)
target_include_directories(${TOOLS_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${TOOLS_LIB_NAME} PUBLIC ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES})

install(TARGETS ${TOOLS_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# python_tools
set(TOOLS_LIB_NAME _python_tools)
configure_file(src/python_tools.cpp.in python_tools.cpp @ONLY)
add_library(${TOOLS_LIB_NAME} SHARED ${CMAKE_CURRENT_BINARY_DIR}/python_tools.cpp)
target_link_libraries(${TOOLS_LIB_NAME} PRIVATE moveit_python_tools)


set(MOVEIT_PYTHON_MODULE_PATH moveit/python_tools)
set_target_properties(${TOOLS_LIB_NAME} ${CORE_LIB_NAME} ${STAGES_LIB_NAME}
	PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	PREFIX "")

install(TARGETS ${TOOLS_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
)
add_custom_command(TARGET ${TOOLS_LIB_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:${TOOLS_LIB_NAME}>" "${CMAKE_CURRENT_SOURCE_DIR}/../src/${MOVEIT_PYTHON_MODULE_PATH}/$<TARGET_FILE_NAME:${TOOLS_LIB_NAME}>"
	COMMENT "Creating symlink to ${TOOLS_LIB_NAME} in source space"
)


# core
set(CORE_LIB_NAME _core)
set(INCLUDES ${PROJECT_SOURCE_DIR}/include/moveit/python/task_constructor)
configure_file(src/core.cpp.in core.cpp @ONLY)
add_library(${CORE_LIB_NAME} SHARED
	${INCLUDES}/properties.h

	src/properties.cpp
	src/solvers.cpp
	src/core.cpp
	${CMAKE_CURRENT_BINARY_DIR}/core.cpp
)
target_include_directories(${CORE_LIB_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(${CORE_LIB_NAME} PUBLIC ${PROJECT_NAME} moveit_python_tools)

# stages
set(STAGES_LIB_NAME _stages)
configure_file(src/stages.cpp.in stages.cpp @ONLY)
add_library(${STAGES_LIB_NAME} SHARED
	src/stages.cpp
	${CMAKE_CURRENT_BINARY_DIR}/stages.cpp
)
target_link_libraries(${STAGES_LIB_NAME} ${CORE_LIB_NAME} ${PROJECT_NAME}_stages)


set(MOVEIT_PYTHON_MODULE_PATH moveit/task_constructor)
set_target_properties(${CORE_LIB_NAME} ${STAGES_LIB_NAME}
	PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	PREFIX "")

install(TARGETS ${CORE_LIB_NAME} ${STAGES_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
)


# Add symlinks from source space to devel-space libs
# This is necessary to find them from devel-space too
foreach(tgt ${CORE_LIB_NAME} ${STAGES_LIB_NAME})
add_custom_command(TARGET ${tgt} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:${tgt}>" "${CMAKE_CURRENT_SOURCE_DIR}/../src/${MOVEIT_PYTHON_MODULE_PATH}/$<TARGET_FILE_NAME:${tgt}>"
	COMMENT "Creating symlink to ${tgt} in source space"
)
endforeach()
