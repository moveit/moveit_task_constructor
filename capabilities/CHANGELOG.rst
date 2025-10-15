^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_capabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Replace deprecated ament_target_dependencies()
* Migrate to new arguments for static_transform_publisher
* Add missing include fmt/ranges.h (`#712 <https://github.com/moveit/moveit_task_constructor/issues/712>`_)
* Reset preempt_request after canceling execution (`#689 <https://github.com/moveit/moveit_task_constructor/issues/689>`_)
* Let Task::preempt() cancel execute() (`#684 <https://github.com/moveit/moveit_task_constructor/issues/684>`_)
* Add missing include fmt/ranges.h (`#712 <https://github.com/moveit/moveit_task_constructor/issues/712>`_)
* Provide action feedback during task execution (`#653 <https://github.com/moveit/moveit_task_constructor/issues/653>`_)
* Use .hpp headers (`#641 <https://github.com/moveit/moveit_task_constructor/issues/641>`_)
* Silent error "Found empty JointState message"
* Simplify formatting code with https://github.com/fmtlib (`#499 <https://github.com/moveit/moveit_task_constructor/issues/499>`_)
* Print warning if no controllers are configured for trajectory execution (`#514 <https://github.com/moveit/moveit_task_constructor/issues/514>`_)
* Fix Solution::fillMessage() (`#432 <https://github.com/moveit/moveit_task_constructor/issues/432>`_)
* Add property trajectory_execution_info (`#355 <https://github.com/moveit/moveit_task_constructor/issues/355>`_, `#502 <https://github.com/moveit/moveit_task_constructor/issues/502>`_)
* ExecuteTaskSolutionCapability: Reject new goals when busy (`#496 <https://github.com/moveit/moveit_task_constructor/issues/496>`_)
* ExecuteTaskSolutionCapability: Rename goalCallback() -> execCallback()
* Replace namespace robot\_[model|state] with moveit::core
* Use pluginlib consistently (`#463 <https://github.com/moveit/moveit_task_constructor/issues/463>`_)
* remove underscore from public members in MotionPlanResponse (`#426 <https://github.com/moveit/moveit_task_constructor/issues/426>`_)
* Rely on CXXFLAGS definition from moveit_common package
* Fix compiler warnings
* Alphabetize package.xml's and CMakeLists
* execute_task_solution_capability: check for canceling request before canceling the goal handle (`#321 <https://github.com/moveit/moveit_task_constructor/issues/321>`_)
* ROS 2 Migration (`#170 <https://github.com/moveit/moveit_task_constructor/issues/170>`_)
* Contributors: AndyZe, Cihat Kurtuluş Altıparmak, Dhruv Patel, Henning Kayser, Jafar Abdi, JafarAbdi, Marq Rasmussen, Michael Görner, Peter David Fagan, Robert Haschke, Sebastian Jahr

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
