#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(${PROJECT_NAME}-test test_pickplace_demo.cpp)
  target_link_libraries(${PROJECT_NAME}-test $<TARGET_OBJECTS:pick_place_task> ${catkin_LIBRARIES} gtest_main)

  add_rostest(pickplace_demo.test)
endif()
