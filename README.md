# MoveIt! Task Constructor Framework

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of *multiple interdependent* subtasks.
It draws on the planning capabilities of [MoveIt!](https://moveit.ros.org/) to solve individual subproblems in black-box *planning stages*.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using *containers*, allowing for sequential as well as parallel compositions.
For more details, please refer to the associated [ICRA 2019 publication](https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf).

**The framework is still under development. The API is unstable and incomplete.**

**Feedback and contributions are very welcome.**

The current aim is to replace MoveIt's old pick&place pipeline and provide a *transparent mechanism* to enable and debug complex motion sequences.

The software repository is compatible to MoveIt's Melodic branch, which also works on Kinetic (when compiled from source).
