#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(pick_place_test pick_place.test pick_place_test.cpp)
  target_link_libraries(pick_place_test ${PROJECT_NAME}_pick_place_task ${catkin_LIBRARIES})
endif()
