add_library(${PROJECT_NAME}_subtasks
	move.cpp
	current_state.cpp
	gripper.cpp
	generate_grasp_pose.cpp
	cartesian_position_motion.cpp

	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/subtasks/move.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/subtasks/current_state.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/subtasks/gripper.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/subtasks/generate_grasp_pose.h
	${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/subtasks/cartesian_position_motion.h
)
target_link_libraries(${PROJECT_NAME}_subtasks ${catkin_LIBRARIES})
