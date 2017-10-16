add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/cartesian_position_motion.h
	${PROJECT_INCLUDE}/stages/compute_ik.h
	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/gripper.h
	${PROJECT_INCLUDE}/stages/modify_planning_scene.h
	${PROJECT_INCLUDE}/stages/move.h

	cartesian_position_motion.cpp
	current_state.cpp
	compute_ik.cpp
	generate_grasp_pose.cpp
	gripper.cpp
	modify_planning_scene.cpp
	move.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
