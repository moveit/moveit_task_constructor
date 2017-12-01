add_library(${PROJECT_NAME}
	properties.cpp
	storage.cpp
	stage.cpp
	container.cpp
	task.cpp
	introspection.cpp

	${PROJECT_INCLUDE}/utils.h
	${PROJECT_INCLUDE}/properties.h
	${PROJECT_INCLUDE}/storage.h
	${PROJECT_INCLUDE}/stage.h
	${PROJECT_INCLUDE}/stage_p.h
	${PROJECT_INCLUDE}/container.h
	${PROJECT_INCLUDE}/container_p.h
	${PROJECT_INCLUDE}/task.h
	${PROJECT_INCLUDE}/introspection.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_subdirectory(stages)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
