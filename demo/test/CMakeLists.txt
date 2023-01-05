#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pick_place_test pick_place_test.cpp)
  target_link_libraries(pick_place_test ${PROJECT_NAME}_pick_place_task)
endif()
