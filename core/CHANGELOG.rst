^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2023-03-06)
------------------
* MoveRelative: Allow backwards operation for joint-space delta (`#436 <https://github.com/ros-planning/moveit_task_constructor/issues/436>`_)
* ComputeIK: Limit collision checking to JMG (`#428 <https://github.com/ros-planning/moveit_task_constructor/issues/428>`_)
* Fix: Fetch pybind11 submodule if not yet present
* Contributors: Robert Haschke

0.1.2 (2023-02-24)
------------------
* Remove moveit/__init__.py during .deb builds to avoid installation conflicts
* MultiPlanner solver (`#429 <https://github.com/ros-planning/moveit_task_constructor/issues/429>`_): a planner that tries multiple planners in sequence

  * CartesianPath: Deprecate redundant property setters
  * PlannerInterface: provide "timeout" property
  * PlannerInterface: provide setters for properties
* JointInterpolation: fix timeout handling
* Contributors: Robert Haschke

0.1.1 (2023-02-15)
------------------
* Resort to MoveIt's python tools
* Provide ComputeIK.ik_frame as full PoseStamped
* Fixed build farm issues

  * Removed unused eigen_conversions includes
  * Fixed odr compiler warning on Buster
  * Fixed missing dependency declarations
* Contributors: Robert Haschke

0.1.0 (2023-02-02)
------------------
* Initial release
* Contributors: Michael Görner, Robert Haschke, Captain Yoshi, Christian Petersmeier, Henning Kayser, Jafar Abdi, Tyler Weaver
