add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/modify_planning_scene.h
	${PROJECT_INCLUDE}/stages/fix_collision_objects.h

	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/compute_ik.h

	${PROJECT_INCLUDE}/stages/connect.h
	${PROJECT_INCLUDE}/stages/move_to.h
	${PROJECT_INCLUDE}/stages/move_relative.h

	${PROJECT_INCLUDE}/stages/simple_grasp.h
	${PROJECT_INCLUDE}/stages/pick.h

	${PROJECT_INCLUDE}/stages/cartesian_position_motion.h
	${PROJECT_INCLUDE}/stages/gripper.h
	${PROJECT_INCLUDE}/stages/move.h

	modify_planning_scene.cpp
	fix_collision_objects.cpp

	current_state.cpp
	generate_grasp_pose.cpp
	compute_ik.cpp

	connect.cpp
	move_to.cpp
	move_relative.cpp

	simple_grasp.cpp
	pick.cpp

	cartesian_position_motion.cpp
	gripper.cpp
	move.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
