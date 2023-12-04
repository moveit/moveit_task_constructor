How To Use MTC (C++)
====================

Initializing a MTC Task
-----------------------

The top-level planning problem is specified as a MTC Task and the subproblems which are specified by Stages are added to the MTC task object.

.. code-block:: c++

  auto node = std::make_shared<rclcpp::Node>();
  auto task = std::make_unique<moveit::task_constructor::Task>();
  task->loadRobotModel(node);
  // Set controllers used to execute robot motion
  task->setProperty("trajectory_execution_info", "joint_trajectory_controller gripper_controller");


Adding containers and stages to a MTC Task
-------------------------------------------

Adding a stage to MTC task -

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
  task->add(std::move(current_state));

Containers derive from Stage and hence containers can be added to MTC task similarly

.. code-block:: c++

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick Object");
  // TODO: Add stages to the container before adding the container to MTC task
  task->add(std::move(container));

Remember, processing starts from generator stages, expands via propagators, and finally connects partial solution sequences via connector stages.
Therefore, there exist limitations on how stages can be added to the task. For example, two generator stages cannot occur in sequence as they would both attempt to *write* into their interfaces, but none of them is actually *reading*. Same applies for two connectors in a row: they would both attempt to read, while no stage is actually writing.
The compatibility of stages is automatically checked once before planning by ``Task::init()``.

Setting planning solvers
------------------------

Stages that does motion planning need solver information.

Solvers available in MTC

* PipelinePlanner - Uses MoveIt's planning pipeline

* JointInterpolation - Interpolates between the start and goal joint states. It does not support complex motions.

* CartesianPath - Moves the end effector in a straight line in Cartesian space.

Code Example on how to initialize the solver

.. code-block:: c++

  const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
      node, "ompl", "RRTConnectkConfigDefault");
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

These solvers will be passed into stages like MoveTo, MoveRelative and Connect.

Setting Properties
------------------

| Each MTC stage has configurable properties. Example - planning group, timeout, goal state, etc.
| Properties of different types can be set using the function below.

.. code-block:: c++

  void setProperty(const std::string& name, const boost::any& value);

| Children stages can easily inherit properties from their parents, thus reducing the configuration overhead.

Cost calculator for Stages
---------------------------

CostTerm is the basic interface to compute costs for solutions for MTC stages.

CostTerm implementations available in MTC

* Constant - Adds a constant cost to each solution

* PathLength - Cost depends on trajectory length with optional weight for different joints

* TrajectoryDuration - Cost depends on execution duration of the whole trajectory

* TrajectoryCostTerm - Cost terms that only work on SubTrajectory solutions 

* LambdaCostTerm - Pass in a lambda expression to calculate cost

* DistanceToReference - Cost depends on weighted joint space distance to a reference point

* LinkMotion - Cost depends on length of Cartesian trajectory of a link

* Clearance - Cost is inverse of distance to collision

Example code on how to set CostTerm using LamdaCostTerm

.. code-block:: c++

  stage->setCostTerm(moveit::task_constructor::LambdaCostTerm(
        [](const moveit::task_constructor::SubTrajectory& traj) { return 100 * traj.cost(); }));

All stages provided by MTC have default cost terms. Stages which produce trajectories as solutions usually use path length to calculate cost.

Planning and Executing a MTC Task
---------------------------------

Planning MTC task will return a MoveItErrorCode.

.. code-block:: c++

  auto error_code = task.plan()

After planning, extract the first successful solution and pass it to the execute function. This will create an ``execute_task_solution`` action client and the action server resides in ``execute_task_solution_capability`` plugin provided by MTC.
The plugin extends MoveGroupCapability. It constructs a MotionPlanRequest from the MTC solution and uses MoveIt's PlanExecution to actuate the robot.

.. code-block:: c++

  auto result = task.execute(*task.solutions().front());
