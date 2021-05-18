#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(pick_place_test pick_place_test.cpp)
  target_link_libraries(pick_place_test $<TARGET_OBJECTS:pick_place_task> ${catkin_LIBRARIES} gtest_main)

  add_rostest(pick_place.test)
endif()
