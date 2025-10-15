^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Mandatory yaml dependency
* Increase minimum required CMake version to 3.16 supported by Ubuntu 20.04
* clang-format-14
* clang-tidy fixes: use uint8_t enums
* Simplify formatting code with https://github.com/fmtlib (`#499 <https://github.com/moveit/moveit_task_constructor/issues/499>`_)
* Drop Kinetic support
* Fix discontinuity in trajectory (`#485 <https://github.com/moveit/moveit_task_constructor/issues/485>`_)
* Cleanup debug output
* Add more debugging output
* Hide button to show rviz-based task construction (`#492 <https://github.com/moveit/moveit_task_constructor/issues/492>`_)
* Fix Qt 5.15 deprecation warnings
* Limit time to wait for execute_task_solution action server
* Replace namespace robot\_[model|state] with moveit::core
* Use pluginlib consistently (`#463 <https://github.com/moveit/moveit_task_constructor/issues/463>`_)
* Contributors: Michael Görner, Robert Haschke

0.1.3 (2023-03-06)
------------------

0.1.2 (2023-02-24)
------------------

0.1.1 (2023-02-15)
------------------
* Remove unused eigen_conversions includes
* Contributors: Robert Haschke

0.1.0 (2023-02-02)
------------------
* Initial release
* Contributors: Robert Haschke, Michael Görner
