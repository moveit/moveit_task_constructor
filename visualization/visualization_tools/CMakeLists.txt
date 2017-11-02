set(MOVEIT_LIB_NAME moveit_task_visualization_tools)

set(HEADERS
  include/moveit_task_constructor/visualization_tools/task_solution_visualization.h
  include/moveit_task_constructor/visualization_tools/task_solution_panel.h
)

add_library(${MOVEIT_LIB_NAME}
  src/task_solution_visualization.cpp
  src/task_solution_panel.cpp
  ${HEADERS}
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC include)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
