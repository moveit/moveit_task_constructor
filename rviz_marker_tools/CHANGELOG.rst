^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_marker_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Simplify include of tf2_eigen.hpp
* Replace deprecated ament_target_dependencies()
* Increase minimum required CMake version to 3.16 supported by Ubuntu 20.04
* clang-tidy fixes: use uint8_t enums
* Ignore Debian-specific catkin_lint error around urdfdom_headers (`#614 <https://github.com/moveit/moveit_task_constructor/issues/614>`_)
* clean up dependencies for rviz_marker_tools (`#610 <https://github.com/moveit/moveit_task_constructor/issues/610>`_)
* rviz_marker_tools: add missing dependency on urdfdom
* rviz_marker_tools: drop rviz dependency
* ROS 2 Migration (`#170 <https://github.com/moveit/moveit_task_constructor/issues/170>`_)
* Contributors: AndyZe, Henning Kayser, JafarAbdi, Jochen Sprickerhof, Michael Görner, Robert Haschke

0.1.3 (2023-03-06)
------------------

0.1.2 (2023-02-24)
------------------
* Fix marker creation: allow zero scale for geometric shapes (`#430 <https://github.com/ros-planning/moveit_task_constructor/issues/430>`_)
* Contributors: Robert Haschke

0.1.1 (2023-02-15)
------------------

0.1.0 (2023-02-02)
------------------
* Initial release
* Contributors: Robert Haschke, Michael Görner
