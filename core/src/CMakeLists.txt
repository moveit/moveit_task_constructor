add_library(${PROJECT_NAME}
	storage.cpp
	stage.cpp
	container.cpp
	task.cpp
	introspection.cpp

	${PROJECT_INCLUDE}/utils.h
	${PROJECT_INCLUDE}/storage.h
	${PROJECT_INCLUDE}/stage.h
	${PROJECT_INCLUDE}/stage_p.h
	${PROJECT_INCLUDE}/container.h
	${PROJECT_INCLUDE}/container_p.h
	${PROJECT_INCLUDE}/task.h
	${PROJECT_INCLUDE}/introspection.h
)
target_include_directories(${PROJECT_NAME}
	PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

add_subdirectory(stages)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
