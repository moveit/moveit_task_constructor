add_library(${PROJECT_NAME}_stages
	move.cpp
	current_state.cpp
	gripper.cpp
	generate_grasp_pose.cpp
	cartesian_position_motion.cpp

	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/stages/move.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/stages/current_state.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/stages/gripper.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/stages/generate_grasp_pose.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/stages/cartesian_position_motion.h
)
target_link_libraries(${PROJECT_NAME}_stages ${catkin_LIBRARIES})
