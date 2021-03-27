add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/modify_planning_scene.h
	${PROJECT_INCLUDE}/stages/fix_collision_objects.h

	${PROJECT_INCLUDE}/stages/current_state.h
	${PROJECT_INCLUDE}/stages/fixed_state.h
	${PROJECT_INCLUDE}/stages/fixed_cartesian_poses.h
	${PROJECT_INCLUDE}/stages/generate_pose.h
	${PROJECT_INCLUDE}/stages/generate_grasp_pose.h
	${PROJECT_INCLUDE}/stages/generate_place_pose.h
	${PROJECT_INCLUDE}/stages/compute_ik.h
	${PROJECT_INCLUDE}/stages/passthrough.h
	${PROJECT_INCLUDE}/stages/predicate_filter.h

	${PROJECT_INCLUDE}/stages/connect.h
	${PROJECT_INCLUDE}/stages/move_to.h
	${PROJECT_INCLUDE}/stages/move_relative.h

	${PROJECT_INCLUDE}/stages/simple_grasp.h
	${PROJECT_INCLUDE}/stages/pick.h

	modify_planning_scene.cpp
	fix_collision_objects.cpp

	current_state.cpp
	fixed_state.cpp
	fixed_cartesian_poses.cpp
  grasp_provider_base.cpp
						${PROJECT_INCLUDE}/stages/grasp_provider_base.h
  place_provider_base.cpp
						${PROJECT_INCLUDE}/stages/place_provider_base.h
	generate_pose.cpp
	generate_grasp_pose.cpp
	generate_place_pose.cpp
	compute_ik.cpp
	passthrough.cpp
	predicate_filter.cpp

	connect.cpp
	move_to.cpp
	move_relative.cpp

	simple_grasp.cpp
	pick.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(${PROJECT_NAME}_providers
	${PROJECT_INCLUDE}/stages/grasp_provider.h
	${PROJECT_INCLUDE}/stages/place_provider.h
						${PROJECT_INCLUDE}/stages/place_provider_base.h
						${PROJECT_INCLUDE}/stages/grasp_provider_base.h
	grasp_provider.cpp
	place_provider.cpp
)
target_link_libraries(${PROJECT_NAME}_providers ${PROJECT_NAME} ${PROJECT_NAME}_stages ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_providers
				ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
				LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
