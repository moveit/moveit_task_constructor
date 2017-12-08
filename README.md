# moveit_task_constructor
An approach to forward multi-step manipulation planning

**The framework is currently under development. The API is unstable and incomplete.**

**Feedback is very welcome.**

This project enables the user to specify and plan *complex manipulation actions* in terms of successive *planning stages*.

Individual stages compute robot trajectories relative to their expected start *or end*.
The resulting planning pipeline, i.e. Task, extends different candidate trajectories from key states (Generator stages)
until it generated feasible trajectories that extend through all stages.

The current aim is to replace MoveIt's old pick&place pipeline and provide a *transparent mechanism* to enable and debug complex motion sequences.
