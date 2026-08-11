add_library(${PROJECT_NAME}_tasks
	${PROJECT_INCLUDE}/tasks/pick_place_task.h
	pick_place_task.cpp
)
target_link_libraries(${PROJECT_NAME}_tasks ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_tasks
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
