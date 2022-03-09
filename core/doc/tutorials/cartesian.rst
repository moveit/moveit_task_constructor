.. _subsec-tut-cartesian:

Cartesian
---------

The following example demonstrates how to compute a simple point to point motion
plan using the moveit task constructor. You can take a look at the full
source code here:
:download:`Source <../../../demo/scripts/cartesian.py>`

First, lets make sure we specify the planning group and the
end effector that we want to use.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut1]
    :end-before: [cartesianTut1]

The moveit task constructor provides different planners.
We will use the ``CartesianPath`` and ``JointInterpolation``
planners for this example.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut2]
    :end-before: [cartesianTut2]

Lets start by initializing a task and adding the current
planning scene state and robot state to it.
This will be the starting state for our motion plan.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut3]
    :end-before: [cartesianTut3]

To compute a relative motion in cartesian space, we can use
the ``MoveRelative`` stage. Specify the planning group and
frame relative to which you want to carry out the motion.
the relative direction can be specified by a ``Vector3Stamped``
geometry message.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveRelative]
    :end-before: [initAndConfigMoveRelative]

Similarly we can move along a different axis.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut4]
    :end-before: [cartesianTut4]

The ``MoveRelative`` stage also offers an interface to
``Twist`` messages, allowing to specify rotations.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut5]
    :end-before: [cartesianTut5]

Lastly, we can compute linear movements in cartesian space
by providing offsets in joint space.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [cartesianTut6]
    :end-before: [cartesianTut6]

If we want to specify goals instead of directions,
we can use the ``MoveTo`` stage. In the following example
we use simple joint interpolation to move the robot to
a named pose. the named pose is defined in the urdf of
the robot configuration.

.. literalinclude:: ../../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveTo]
    :end-before: [initAndConfigMoveTo]

Lastly, we invoke the planning mechanism that traverses
the task hierarchy for us and compute a valid motion plan.
At this point, when you run this script you are able to
inspect the solutions of the individual stages in the rviz
mtc panel.
