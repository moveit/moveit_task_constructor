^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_task_constructor_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2025-10-15)
------------------
* Simplify include of tf2_eigen.hpp
* Avoid duplicate scenes in Solution.msg from generator stages (`#639 <https://github.com/moveit/moveit_task_constructor/issues/639>`_)
* Allow max Cartesian link speed in PlannerInterface (`#277 <https://github.com/moveit/moveit_task_constructor/issues/277>`_)
* Replace deprecated ament_target_dependencies()
* Enable collisions visualizations (`#708 <https://github.com/moveit/moveit_task_constructor/issues/708>`_)
* LimitSolutions wrapper stage (`#710 <https://github.com/moveit/moveit_task_constructor/issues/710>`_)
* Improve code documentation
* CI: Fix Noble builds
* Pick with custom max_velocity_scaling_factor during approach+lift
* Modernize declaration of compile options
* Factor out Python property handling to allow for reuse in custom Python wrappers
* Let Task::preempt() cancel execute() (`#684 <https://github.com/moveit/moveit_task_constructor/issues/684>`_)
* Fix clamping of joint constraints (`#665 <https://github.com/moveit/moveit_task_constructor/issues/665>`_)
* Correctly report failures instead of issueing console warnings
* Increase minimum required CMake version to 3.16 supported by Ubuntu 20.04
* Python API: Allow passing a task's introspection object to SolutionBase::toMsg()
* clang-format-14
* Use .hpp headers (`#641 <https://github.com/moveit/moveit_task_constructor/issues/641>`_)
* Add support for GenerateRandomPose
* python: Add Task::setRobotModel
* Add path_constraints property to Connect stage
* provide a fmt wrapper (`#615 <https://github.com/moveit/moveit_task_constructor/issues/615>`_)
* Update API: JumpThreshold -> CartesianPrecision (`#611 <https://github.com/moveit/moveit_task_constructor/issues/611>`_)
* Reduce stop time due to preempt (`#598 <https://github.com/moveit/moveit_task_constructor/issues/598>`_)
* Add unittest for `#581 <https://github.com/moveit/moveit_task_constructor/issues/581>`_
* Fix early planning preemption (`#597 <https://github.com/moveit/moveit_task_constructor/issues/597>`_)
* MoveRelative: fix segfault on empty trajectory (`#595 <https://github.com/moveit/moveit_task_constructor/issues/595>`_)
* MoveRelative: handle equal min/max distance (`#593 <https://github.com/moveit/moveit_task_constructor/issues/593>`_)
* CI: skip python tests
* Adapt python scripts to ROS2 API changes
* Reenable python bindings
* Cleanup unit tests and allow them to run via both, cmdline and pytest
* Connect: Relax validity check of reached end state
* Unify Python demo scripts
* Switch shebang to python3
* Silence gcc's overloaded-virtual warnings
* Fix flaky IK in asan unit test: increase timeout
* Fix failing unittests: remove static executor
* Add property to enable/disable pruning at runtime (`#590 <https://github.com/moveit/moveit_task_constructor/issues/590>`_)
* Disable pruning by default
* test_pruning.cpp: Add new test
* test_pruning.cpp: Extend test to ParallelContainer
* PassThrough: cleanup unused headers
* Avoid segfault if TimeParameterization is not set
* CartesianPath: allow ik_frame definition if start and end are given as joint-space poses
* Generalize utils::getRobotTipForFrame() to return error_msg instead of calling markAsFailure() on a solution
* ComputeIK: Allow additional constraints for filtering solutions (`#464 <https://github.com/moveit/moveit_task_constructor/issues/464>`_)
* Expose MultiPlanner to Python (`#474 <https://github.com/moveit/moveit_task_constructor/issues/474>`_)
* Add unittest cartesianCollisionMinMaxDistance (`#538 <https://github.com/moveit/moveit_task_constructor/issues/538>`_)
* Connect: Relax validity check of reached end state (`#542 <https://github.com/moveit/moveit_task_constructor/issues/542>`_)
* Simplify formatting code with https://github.com/fmtlib (`#499 <https://github.com/moveit/moveit_task_constructor/issues/499>`_)
* Add NoOp stage (`#534 <https://github.com/moveit/moveit_task_constructor/issues/534>`_)
* ModifyPlanningScene: check state for collisions
* Improve TypeError exceptions
* Switch to package py_binding_tools
* Add ability to move CollisionObjects (`#567 <https://github.com/moveit/moveit_task_constructor/issues/567>`_)
* Simplify rclcpp::Node creation
* Improve description of max_distance property of Connect stage (`#564 <https://github.com/moveit/moveit_task_constructor/issues/564>`_)
* Add Generator::spawn(from, to, trajectory) variant (`#546 <https://github.com/moveit/moveit_task_constructor/issues/546>`_)
* Cosmetic fixes
* Fix Solution::fillMessage() (`#432 <https://github.com/moveit/moveit_task_constructor/issues/432>`_)
* Fix generation of Solution msg: consider backward operation
* Propagate errors from planners to solution comment (`#525 <https://github.com/moveit/moveit_task_constructor/issues/525>`_)
* Add planner info to comments (`#523 <https://github.com/moveit/moveit_task_constructor/issues/523>`_)
* Fix MTC unittests for new pipeline refactoring (`#515 <https://github.com/moveit/moveit_task_constructor/issues/515>`_)
* Update to the more recent JumpThreshold API (`#506 <https://github.com/moveit/moveit_task_constructor/issues/506>`_)
* JointInterpolationPlanner: Check joint bounds (`#505 <https://github.com/moveit/moveit_task_constructor/issues/505>`_)
* Add property trajectory_execution_info (`#355 <https://github.com/moveit/moveit_task_constructor/issues/355>`_, `#502 <https://github.com/moveit/moveit_task_constructor/issues/502>`_)
* Clear JointStates in scene diff (`#504 <https://github.com/moveit/moveit_task_constructor/issues/504>`_)
* Set a non-infinite default timeout in CurrentState stage (`#491 <https://github.com/moveit/moveit_task_constructor/issues/491>`_)
* Add GenerateRandomPose stage (`#166 <https://github.com/moveit/moveit_task_constructor/issues/166>`_)
* Add planner_id to SubTrajectory info (`#490 <https://github.com/moveit/moveit_task_constructor/issues/490>`_)
* Remove display_motion_plans and publish_planning_requests properties (`#489 <https://github.com/moveit/moveit_task_constructor/issues/489>`_)
* Enable parallel planning with PipelinePlanner (`#450 <https://github.com/moveit/moveit_task_constructor/issues/450>`_)
* GenerateGraspPose: Expose rotation_axis as property (`#535 <https://github.com/moveit/moveit_task_constructor/issues/535>`_)
* Connect: ensure end-state matches goal state (`#532 <https://github.com/moveit/moveit_task_constructor/issues/532>`_)
* Fix discontinuity in trajectory (`#485 <https://github.com/moveit/moveit_task_constructor/issues/485>`_)
* Adaptions for https://github.com/ros-planning/moveit/pull/3534
* Cleanup debug output
* Fix duplicate solutions
* printPendingPairs(os) -> os<<pendingPairsPrinter()
* Fix leaking of failures into enumerated solutions
* Add more debugging output
* Unit tests for `#485 <https://github.com/moveit/moveit_task_constructor/issues/485>`_
* DelayingWrapper stage to delay solution shipping in unit tests
* Simplify tests
* Skip Fallbacks::replaceImpl() when already correctly initialized (`#494 <https://github.com/moveit/moveit_task_constructor/issues/494>`_)
* Fix demos (`#493 <https://github.com/moveit/moveit_task_constructor/issues/493>`_)
* Limit time to wait for execute_task_solution action server
* Replace namespace robot\_[model|state] with moveit::core
* MPS: fixup processCollisionObject
* Merge PR `#460 <https://github.com/moveit/moveit_task_constructor/issues/460>`_: improvements to ModifyPlanningScene stage
* Gracefully handle NULL robot_trajectory (`#469 <https://github.com/moveit/moveit_task_constructor/issues/469>`_)
* introspection: remove any invalid ROS-name chars from hostname (`#465 <https://github.com/moveit/moveit_task_constructor/issues/465>`_)
* Fix SolutionBase::fillMessage(): also write start_scene
* Fix add/remove object in backward operation
* Add python binding for ModifyPlanningScene::removeObject
* ComputeIK: update RobotState before calling setFromIK()
* Use pluginlib consistently (`#463 <https://github.com/moveit/moveit_task_constructor/issues/463>`_)
* Expose argument of PipelinePlanner's constructor to Python (`#462 <https://github.com/moveit/moveit_task_constructor/issues/462>`_)
* Fix allowCollisions(object, enable_collision)
* TestModifyPlanningScene
* Basic Move test: MoveRelative + MoveTo
* Add python binding for ModifyPlanningScene::allowCollisions(std::string, bool)
* Add python binding for Task::insert
* Add Stage::explainFailure() (`#445 <https://github.com/moveit/moveit_task_constructor/issues/445>`_)
* Improve documentation (`#431 <https://github.com/moveit/moveit_task_constructor/issues/431>`_)
* JointInterpolationPlanner: pass optional max_effort property along to GripperCommand (`#458 <https://github.com/moveit/moveit_task_constructor/issues/458>`_)
* Task: findChild() and operator[] should directly operate on stages() (`#435 <https://github.com/moveit/moveit_task_constructor/issues/435>`_)
* Remove underscore from public members in MotionPlanResponse (`#426 <https://github.com/moveit/moveit_task_constructor/issues/426>`_)
* ROS 2 Migration (`#170 <https://github.com/moveit/moveit_task_constructor/issues/170>`_)
* Contributors: Abishalini, Abishalini Sivaraman, Ali Haider, AndyZe, Captain Yoshi, Cihat Kurtuluş Altıparmak, Daniel García López, Gauthier Hentz, Henning Kayser, Jafar, Jafar Uruç, JafarAbdi, Mario Prats, Marq Rasmussen, Michael Görner, Michael Wiznitzer, Paul Gesel, Peter David Fagan, Robert Haschke, Sebastian Castro, Sebastian Jahr, Tyler Weaver, VideoSystemsTech, Wyatt Rees

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
