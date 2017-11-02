add_library(${PROJECT_NAME}_stages
	move.cpp
	current_state.cpp
	gripper.cpp
	generate_grasp_pose.cpp
	cartesian_position_motion.cpp

	${PROJECT_INCLUDE}/stages/move.h
	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/gripper.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/cartesian_position_motion.h
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
