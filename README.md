# MoveIt Task Constructor Framework

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of [MoveIt](https://moveit.ros.org/) to solve individual subproblems in black-box *planning stages*.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using *containers*, allowing for sequential as well as parallel compositions.

## Video

[![Video associated with ICRA 2019 paper](https://img.youtube.com/vi/fCORKVYsdDI/0.jpg )](https://www.youtube.com/watch?v=fCORKVYsdDI)

## Roadmap

**Feedback, reports and contributions are very welcome.**

The current roadmap is to replace MoveIt's old pick&place pipeline and provide a *transparent mechanism* to enable and debug complex motion sequences.

Further planned features include

- Entwined planning and execution for early execution, monitoring and code hooks
- Support custom cost terms
- Subsolution blending
- Parallel planning
- Iterative solution improvement

Ideas and requests for other interesting/useful features are welcome.

## Citation

If you use this framework in your project, please cite the associated paper:


    Michael Görner*, Robert Haschke*, Helge Ritter, and Jianwei Zhang,
    MoveIt! Task Constructor for Task-Level Motion Planning,
    International Conference on Robotics and Automation, ICRA 2019, Montreal, Canada.
    [[DOI]](https://doi.org/10.1109/ICRA.2019.8793898) [PDF](https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf).


```plain
@inproceedings{goerner2019mtc,
  title={{MoveIt! Task Constructor for Task-Level Motion Planning}},
  author={Görner, Michael* and Haschke, Robert* and Ritter, Helge and Zhang, Jianwei},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2019}
}
```
