Basic Concepts
==============

The fundamental idea of MTC is that complex motion planning problems can be composed into a set of simpler subproblems. The top-level planning problem is specified as a Task while all subproblems are specified by Stages. Stages can be arranged in any arbitrary order and hierarchy only limited by the individual stages types. The order in which stages can be arranged is restricted by the direction in which results are passed. There are three possible stage types w.r.t. their data flow: generators, propagators, and connector stages:

.. glossary::

	Generators
		compute their results independently of their neighboring stages and pass them in both directions, backwards and forwards. Examples include pose generators, e.g. for grasping or placing, as well as ``ComputeIK``, which computes IK solutions for Cartesian targets. neighboring stages can continue processing from the generated states. The most important generator stage, however, is ``CurrentState``, which provides the current robot state as the starting point for a planning pipeline.

	Propagators
		receive the result of *one* neighboring stage as input, plan towards a goal state, and then propagate their result to the opposite interface site. Propagating stages can receive their input on either interface, begin or end. The flow of information (forwards or backwards) is determined by the input interface of the stage. An example is a stage that computes a Cartesian path based on either a start or a goal state.

	Connectors
		do not propagate any results, but rather attempt to bridge the gap between the resulting states of both its neighboring stages. It receives input states from both, the begin and end interface and attempts to connect them via a suitable motion plan. Obviously, any pair of input states needs to be *compatible*, i.e. their states (including collision and attached objects as well as joint poses) need to match except for those joints that are part of the given planning group.

Processing starts from generator stages, expands via propagators, and finally connects partial solution sequences via connector stages.
Obviously, there exist limitations on how stages can be connected to each other. For example, two generator stages cannot occur in sequence as they would both attempt to *write* into their interfaces, but non of them is actually *reading*. Same applies for two connectors in a row: they would both attempt to read, while no stage is actually writing.
The compatibility of stages is automatically checked once before planning by ``Task::init()``.

To compose a planning pipeline from multiple seemingly independent parts, for example grasping an object and placing it at a new location, one needs to break the typical linear pipeline structure: the place pose is another generator stage (additionally to the ``CurrentState`` stage we are starting from) serving as a seed for the placing sub solution. However, this stage is not completely independent from the grasping sub solution: it should continue where grasping left off, namely with the grasped object attached to the gripper. To convey this state information, the place pose generator should inherit from ``MonitoringGenerator``, which monitors the solutions of another stage in the pipeline in fast-forwards them for further processing in the dependent stage.

In order to hierarchically organize planning pipelines and to allow for reuse of sub pipelines, e.g. for grasping or placing, one can encapsulate stages within various containers.
Stages without children are called primitive stages. We provide three types of containers:

.. glossary::

	Wrappers
		encapsulate a single subordinate stage and modify or filter its results. For example, a filter stage that only accepts solutions of its child stage that satisfy a certain constraint can be realized as a wrapper. Another standard use of this type includes the IK wrapper stage, which generates inverse kinematics solutions based on planning scenes annotated with a pose target property.

	Serial containers
		hold a sequence of subordinate stages and only consider end-to-end solutions as results. An example is a picking motion that consists of a sequence of coherent steps.

	Parallel containers
		consider the solutions of all their children as feasible. Different sub types are available, namely:

		``Alternative`` container
			processes all children simultaneously (currently in a round-robin fashion). For example, one could plan a grasping sequence for a left and right arm in parallel if the actual choice of the arm doesn't matter for the task.

		``Fallback`` container
			processes their children sequentially: only if the current child has exhausted all its solution candidates (and didn't produce any feasible solution), the next child is considered.
			Use this container, e.g. if you prefer grasping with the right arm, but allow falling back to the left if really needed.

		``Merger`` container
			processes their children simultaneously and combine their solutions into an joint solution. It is assumed that children operate on disjoint joint model groups, e.g. the arm and the hand, such that their solution trajectories can be executed in parallel after being merged.

Stages not only support solving motion planning problems. They can also be used for all kinds of state transitions, as for instance modifying the planning scene. Combined with the possibility of using class inheritance it is possible to construct very complex behavior while only relying on a well-structured set of primitive stages.
