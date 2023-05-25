MoveIt Task Constructor (MTC)
=============================

The Task Constructor framework provides a flexible and transparent way
to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of MoveIt to solve individual subproblems
in black-box planning stages.
A common interface, based on MoveIt’s PlanningScene is used to pass solution
hypotheses between stages. The framework enables the hierarchical organization of
basic stages using containers, allowing for sequential as well as parallel compositions.
For more details, please refer to the associated `ICRA 2019 publication <https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf>`_.


Organization of the documentation
---------------------------------

- :ref:`sec-tutorials` provide examples how to setup your task pipeline.
  Start with :ref:`subsec-tut-firststeps` if you are new to MTC.
- :ref:`sec-concepts` discuss the architecture and terminology of MTC on a fairly high level.
- :ref:`sec-howtoguides` help solving specific problems and use cases.
- The :ref:`sec-api` provides quick access to available classes, functions, and their parameters.

.. toctree::
  :maxdepth: 2
  :hidden:

  tutorials/index
  concepts
  howto
  api
  troubleshooting
