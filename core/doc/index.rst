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


How the documentation is organized
----------------------------------

- :ref:`sec-tutorials` guide you
  through the initial learning process of setting up a task pipeline.
  Take a look at the :ref:`first steps <subsec-tut-firststeps>` if you
  are new to the moveit task constructor.

- :ref:`sec-concepts` discuss the architecture and terminology
  of the moveit task constructor on a fairly high level.

- :ref:`sec-howtoguides` help you to solve
  specific problems and use cases you might encounter.

- The :ref:`sec-api` provides
  technical details of the python package. You may
  look up definitions here and should already
  have a basic understanding of the key concepts.

.. toctree::
  :maxdepth: 2
  :hidden:

  tutorials
  concepts
  howtoguides
  reference
