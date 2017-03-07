add_executable(plan_pick plan_pick_ur5.cpp)
target_link_libraries(plan_pick ${PROJECT_NAME}_subtasks ${PROJECT_NAME})
