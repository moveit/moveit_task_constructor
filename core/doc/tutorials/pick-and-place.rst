.. _subsec-tut-pick-place:

Pick and Place
--------------

The following tutorial demonstrates how you can use the moveit
task constructor to plan and carry out pick and place movements.

First, lets specify the planning group and the
end effector that you want to use.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut1]
    :end-before: [pickAndPlaceTut1]

Next, we add the object that we want to displace to the
planning scene. To this end, make sure that the planning scene
not already contains such object.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut2]
    :end-before: [pickAndPlaceTut2]

At this point, we are ready to create the task hierarchy.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut3]
    :end-before: [pickAndPlaceTut3]

The pipeline planner encapsulates the moveit interface
to sampling-based geometric motion planners.

.. tip::
    Planning does not proceed linearly from top to bottom.
    Rather, it proceeds from the inside out.
    Connect stages therefore compute a motion plan between
    two previously calculated subordinate solutions.
    For a clear visualization, inspect the rviz mtc panel.

Lets connect the current robot state with the solutions of
the following stages.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut4]
    :end-before: [pickAndPlaceTut4]

To pick the object, we first need to know possible end effector
poses with which we can perform a successful grasp.
For this, we use a ``GenerateGraspPoseStage``.
which essentially spawns poses
with a given ``angle_delta`` in circular fashion around a center
point.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut5]
    :end-before: [pickAndPlaceTut5]

Next, we need to compute the inverse kinematics of the robot arm
for all previously sampled poses. This way we can
rule out solutions that are not feasible due to the robot geometry.
The ``simpleGrasp`` stage combines ik calculation with motion plan
generation for opening and closing the end effector, as well as attaching
the object to the robot an disabling collision.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut6]
    :end-before: [pickAndPlaceTut6]

Lastly, we can insert all the previous steps into the ``Pick``
container stage. At this point we might also specify approach and
lift twists for the robot relative to the object we want to grasp.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut7]
    :end-before: [pickAndPlaceTut7]

Since all the previous stages were chained together via their
constructor arguments, we only need to add the top level ``Pick``
stage to the task hierarchy.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut8]
    :end-before: [pickAndPlaceTut8]

Thats everything we need for picking an object!
Lets find a motion plan to place the object

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut9]
    :end-before: [pickAndPlaceTut9]

Similar to the picking procedure, we define the place task.
First, start with sampling place poses.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut10]
    :end-before: [pickAndPlaceTut10]

Next, wrap the inverse kinematics computation and gripper
movements.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut11]
    :end-before: [pickAndPlaceTut11]

Lastly, add place and retract motions and add the ``Place``
stage to the task hierarchy.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut12]
    :end-before: [pickAndPlaceTut12]

Finally, compute solutions for the task hierarchy and delete
the planner instances.

.. literalinclude:: ../../../demo/scripts/pickplace.py
    :language: python
    :start-after: [pickAndPlaceTut13]
    :end-before: [pickAndPlaceTut13]

At this point, you might inspect the task hierarchy in the mtc rviz
plugin.

.. tip::
    Use the mtc rviz plugin to graphically inspect the solutions
    of individual stages in the task hierarchy.
