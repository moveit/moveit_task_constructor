.. _subsec-tut-firststeps:

First Steps
-----------

The MoveIt Task Constructor package contains several basic examples and
a pick-and-place demo. For all demos you should launch the basic environment:

.. code-block::

    roslaunch moveit_task_constructor_demo demo.launch

Subsequently, you can run the individual demos:

.. code-block::

    rosrun moveit_task_constructor_demo cartesian
    rosrun moveit_task_constructor_demo modular
    roslaunch moveit_task_constructor_demo pickplace.launch

To inspect the task hierarchy, be sure that you selected the correct solution topic
in the reviz moveit task constructor plugin.
