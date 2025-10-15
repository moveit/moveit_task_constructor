^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Fix run.launch.py to load planning pipelines
* Allow max Cartesian link speed in PlannerInterface (`#277 <https://github.com/moveit/moveit_task_constructor/issues/277>`_)
* Replace deprecated ament_target_dependencies()
* Migrate to new arguments for static_transform_publisher
* Add missing dependency
* CI: Fix Noble builds
* Pick with custom max_velocity_scaling_factor during approach+lift
* Fix pick+place: connect should plan both, arm and hand motion
* Fix duplicate rviz node names (`#672 <https://github.com/moveit/moveit_task_constructor/issues/672>`_)
* Adapt to new location of generated parameter include files
* Increase minimum required CMake version to 3.16 supported by Ubuntu 20.04
* clang-tidy fixes: std::endl -> '\n'
* Use .hpp headers (`#641 <https://github.com/moveit/moveit_task_constructor/issues/641>`_)
* examples: add orientation path constraint
* Update API: JumpThreshold -> CartesianPrecision (`#611 <https://github.com/moveit/moveit_task_constructor/issues/611>`_)
* Fix run.launch.py (`#607 <https://github.com/moveit/moveit_task_constructor/issues/607>`_)
* Unify Python demo scripts
* Switch shebang to python3
* Improve comments for pick-and-place task (`#238 <https://github.com/moveit/moveit_task_constructor/issues/238>`_)
* Expose MultiPlanner to Python (`#474 <https://github.com/moveit/moveit_task_constructor/issues/474>`_)
* Example of constrained orientation planning
* Switch to package py_binding_tools
* Enable parallel planning with PipelinePlanner (`#450 <https://github.com/moveit/moveit_task_constructor/issues/450>`_)
* Fix demos (`#493 <https://github.com/moveit/moveit_task_constructor/issues/493>`_)
* Merge PR `#460 <https://github.com/moveit/moveit_task_constructor/issues/460>`_: improvements to ModifyPlanningScene stage
* Fix SolutionBase::fillMessage(): also write start_scene
* Replace rosparam_shortcuts with generate_parameter_library (`#403 <https://github.com/moveit/moveit_task_constructor/issues/403>`_)
* Use moveit_configs_utils for launch files (`#365 <https://github.com/moveit/moveit_task_constructor/issues/365>`_)
* ROS 2 Migration (`#170 <https://github.com/moveit/moveit_task_constructor/issues/170>`_)
* Contributors: AndyZe, Cihat Kurtuluş Altıparmak, Fabian Schuetze, Gauthier Hentz, Henning Kayser, Jafar, JafarAbdi, Julia Jia, Karthik Arumugham, Michael Görner, Robert Haschke, Sebastian Jahr, Stephanie Eng, TipluJacob, VideoSystemsTech

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
