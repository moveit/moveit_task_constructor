add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/cartesian_position_motion.h
	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/gripper.h
	${PROJECT_INCLUDE}/stages/move.h

	cartesian_position_motion.cpp
	current_state.cpp
	generate_grasp_pose.cpp
	gripper.cpp
	move.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
