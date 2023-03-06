^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2023-03-06)
------------------
* Use const reference instead of reference for ros::NodeHandle (`#437 <https://github.com/ros-planning/moveit_task_constructor/issues/437>`_)
* Contributors: Robert Haschke

0.1.2 (2023-02-24)
------------------
* CartesianPath: Deprecate redundant property setters (`#429 <https://github.com/ros-planning/moveit_task_constructor/issues/429>`_)
* Contributors: Robert Haschke

0.1.1 (2023-02-15)
------------------
* Resort to MoveIt's python tools
  * Provide ComputeIK.ik_frame as full PoseStamped
* Fixed build farm issues
  * Fixed missing dependency declarations
* pick_place_task: monitor last state before Connect
  ... to prune solutions as much as possible
* Contributors: Robert Haschke

0.1.0 (2023-02-02)
------------------
* Initial release
* Contributors: Michael Görner, Robert Haschke
