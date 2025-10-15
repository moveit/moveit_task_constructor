^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_capabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Add missing include fmt/ranges.h (`#712 <https://github.com/moveit/moveit_task_constructor/issues/712>`_)
* Provide action feedback during task execution (`#653 <https://github.com/moveit/moveit_task_constructor/issues/653>`_)
* Increase minimum required CMake version to 3.16 supported by Ubuntu 20.04
* Silent error "Found empty JointState message"
* Simplify formatting code with https://github.com/fmtlib (`#499 <https://github.com/moveit/moveit_task_constructor/issues/499>`_)
* Drop Melodic support
* Fix Solution::fillMessage() (`#432 <https://github.com/moveit/moveit_task_constructor/issues/432>`_)
* Add property trajectory_execution_info (`#355 <https://github.com/moveit/moveit_task_constructor/issues/355>`_, `#502 <https://github.com/moveit/moveit_task_constructor/issues/502>`_)
* ExecuteTaskSolutionCapability: Rename goalCallback() -> execCallback()
* Replace namespace robot\_[model|state] with moveit::core
* Use pluginlib consistently (`#463 <https://github.com/moveit/moveit_task_constructor/issues/463>`_)
* Contributors: Dhruv Patel, Michael Görner, Robert Haschke

0.1.3 (2023-03-06)
------------------

0.1.2 (2023-02-24)
------------------

0.1.1 (2023-02-15)
------------------

0.1.0 (2023-02-02)
------------------
* Initial release
* Contributors: Michael Görner, Robert Haschke
