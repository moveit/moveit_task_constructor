cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_visualization)

find_package(fmt REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	moveit_core
	moveit_ros_visualization
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	roscpp
	rviz
)

moveit_build_options()

# rviz transitively includes OGRE headers which break with `-Wall -Werror`
# so isolate these include dirs and add them as SYSTEM include where needed.
set(rviz_OGRE_INCLUDE_DIRS)
foreach(header IN ITEMS OgreRoot.h OgreOverlay.h)
	find_path(include_dir ${header}
		HINTS ${catkin_INCLUDE_DIRS}
		NO_DEFAULT_PATH)
	list(REMOVE_ITEM catkin_INCLUDE_DIRS "${include_dir}")
	list(APPEND rviz_OGRE_INCLUDE_DIRS "${include_dir}")
endforeach()

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

# Qt Stuff
if("${rviz_QT_VERSION}" VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
	macro(qt_wrap_ui)
		qt4_wrap_ui(${ARGN})
	endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
	macro(qt_wrap_ui)
		qt5_wrap_ui(${ARGN})
	endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
	LIBRARIES
		moveit_task_visualization_tools
		motion_planning_tasks_utils
	INCLUDE_DIRS
		visualization_tools/include
	CATKIN_DEPENDS
		moveit_core
		moveit_task_constructor_msgs
		roscpp
		rviz
)

add_subdirectory(visualization_tools)
add_subdirectory(motion_planning_tasks)

install(FILES
	motion_planning_tasks_rviz_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
