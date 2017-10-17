add_executable(plan_pick_ur5 plan_pick_ur5.cpp)
target_link_libraries(plan_pick_ur5 ${PROJECT_NAME}_stages ${PROJECT_NAME})

add_executable(plan_pick_trixi plan_pick_trixi.cpp)
target_link_libraries(plan_pick_trixi ${PROJECT_NAME}_stages ${PROJECT_NAME})
