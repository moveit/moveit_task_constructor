add_executable(plan_pick_ur5 plan_pick_ur5.cpp)
target_link_libraries(plan_pick_ur5 ${PROJECT_NAME}_stages)

add_executable(plan_pick_pr2 plan_pick_pr2.cpp)
target_link_libraries(plan_pick_pr2 ${PROJECT_NAME}_stages)

add_executable(plan_pick_pa10 plan_pick_pa10.cpp)
target_link_libraries(plan_pick_pa10 ${PROJECT_NAME}_stages)
