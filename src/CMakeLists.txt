add_library(${PROJECT_NAME}
	storage.cpp
	stage.cpp
	container.cpp
	task.cpp
	introspection.cpp

	stage_p.h
	container_p.h

	${PROJECT_INCLUDE}/utils.h
	${PROJECT_INCLUDE}/cost_queue.h
	${PROJECT_INCLUDE}/storage.h
	${PROJECT_INCLUDE}/stage.h
	${PROJECT_INCLUDE}/container.h
	${PROJECT_INCLUDE}/task.h
	${PROJECT_INCLUDE}/introspection.h
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(${PROJECT_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_subdirectory(stages)
add_subdirectory(demo)
add_subdirectory(test)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
