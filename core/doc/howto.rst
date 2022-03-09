.. _sec-howtoguides:

How-To Guides
=============

.. _subsec-howto-stage-usage:

Stage Usage
-----------

.. _subsubsec-howto-alternatives:

Alternatives
^^^^^^^^^^^^

Using the ``alternatives`` stage, you can plan for multiple
execution paths.
Download the full example code here: :download:`Source <./../../demo/scripts/alternatives.py>`

.. literalinclude:: ./../../demo/scripts/alternatives.py
    :language: python
    :start-after: [initAndConfigAlternatives]
    :end-before: [initAndConfigAlternatives]

.. _subsubsec-howto-fallbacks:

Fallbacks
^^^^^^^^^

The ``fallbacks`` stage provides alternative motion planners
if planning fails with the primary one.
Download the full example code here: :download:`Source <./../../demo/scripts/fallbacks.py>`

.. literalinclude:: ./../../demo/scripts/fallbacks.py
    :language: python
    :start-after: [initAndConfigFallbacks]
    :end-before: [initAndConfigFallbacks]

.. _subsubsec-howto-merger:

Merger
^^^^^^

Plan and execute sequences in parallel using the ``merger`` stage.
Download the full example code here: :download:`Source <./../../demo/scripts/merger.py>`

.. literalinclude:: ./../../demo/scripts/merger.py
    :language: python
    :start-after: [initAndConfigMerger]
    :end-before: [initAndConfigMerger]

.. _subsubsec-howto-connect:

Connect
^^^^^^^

Connect two stages by finding a motion plan between them.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigConnect]
    :end-before: [initAndConfigConnect]

.. _subsubsec-howto-fix-collision-objects:

FixCollisionObjects
^^^^^^^^^^^^^^^^^^^

Check for collisions and resolve them if applicable.
Download the full example code here: :download:`Source <./../../demo/scripts/fix_collision_objects.py>`

.. literalinclude:: ./../../demo/scripts/fix_collision_objects.py
    :language: python
    :start-after: [initAndConfig]
    :end-before: [initAndConfig]

.. _subsubsec-howto-generate-place-pose:

GeneratePlacePose
^^^^^^^^^^^^^^^^^^^

Sample feasible poses around an object pose.
Considers geometry of primitive object type.
Solutions can be used for inverse
kinematics calculations.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initCollisionObject]
    :end-before: [initCollisionObject]

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigGeneratePlacePose]
    :end-before: [initAndConfigGeneratePlacePose]

.. _subsubsec-howto-generate-grasp-pose:

GenerateGraspPose
^^^^^^^^^^^^^^^^^

Sample poses around an object pose by providing
sample density ``angle_delta``.
Solutions can be used for inverse kinematics
calculations.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigGenerateGraspPose]
    :end-before: [initAndConfigGenerateGraspPose]

.. _subsubsec-howto-generate-pose:

GeneratePose
^^^^^^^^^^^^

Spawn a pose on new solutions of the monitored stage.
Download the full example code here: :download:`Source <./../../demo/scripts/generate_pose.py>`

.. literalinclude:: ./../../demo/scripts/generate_pose.py
    :language: python
    :start-after: [initAndConfigGeneratePose]
    :end-before: [initAndConfigGeneratePose]

.. _subsubsec-howto-pick:

Pick
^^^^

Wraps the task pipeline to execute a pick.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigPick]
    :end-before: [initAndConfigPick]

.. _subsubsec-howto-place:

Place
^^^^^

Wraps the task pipeline to execute a pick.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigPlace]
    :end-before: [initAndConfigPlace]

.. _subsubsec-howto-simplegrasp:

SimpleGrasp
^^^^^^^^^^^

Wraps the pose generation and inverse kinematics
computation for the pick pipeline.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigSimpleGrasp]
    :end-before: [initAndConfigSimpleGrasp]

.. _subsubsec-howto-simpleungrasp:

SimpleUnGrasp
^^^^^^^^^^^^^

Wraps the pose generation and inverse kinematics
computation for the place pipeline.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigSimpleUnGrasp]
    :end-before: [initAndConfigSimpleUnGrasp]

.. _subsubsec-howto-modify-planning-scene:

ModifyPlanningScene
^^^^^^^^^^^^^^^^^^^

Modify the planning scene.
Download the full example code here: :download:`Source <./../../demo/scripts/modify_planning_scene.py>`

.. literalinclude:: ./../../demo/scripts/modify_planning_scene.py
    :language: python
    :start-after: [initAndConfigModifyPlanningScene]
    :end-before: [initAndConfigModifyPlanningScene]

.. _subsubsec-howto-fixed-state:

FixedState
^^^^^^^^^^

Spawn a pre-defined state.
Download the full example code here: :download:`Source <./../../demo/scripts/fixed_state.py>`

.. literalinclude:: ./../../demo/scripts/fixed_state.py
    :language: python
    :start-after: [initAndConfigFixedState]
    :end-before: [initAndConfigFixedState]

.. _subsubsec-howto-compute-ik:

ComputeIK
^^^^^^^^^

Compute the inverse kinematics of the monitored stages'
solution. Be sure to correctly configure the ``target_pose``
property to be derived from the monitored stage as shown
in the example.
Download the full example code here: :download:`Source <./../../demo/scripts/compute_ik.py>`

.. literalinclude:: ./../../demo/scripts/compute_ik.py
    :language: python
    :start-after: [initAndConfigComputeIk]
    :end-before: [initAndConfigComputeIk]

.. _subsubsec-howto-move-to:

MoveTo
^^^^^^

Use planners to compute a motion plan.
Download the full example code here: :download:`Source <../../demo/scripts/cartesian.py>`

.. literalinclude:: ../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveTo]
    :end-before: [initAndConfigMoveTo]

.. _subsubsec-howto-move-relative:

MoveRelative
^^^^^^^^^^^^

Move along a relative offset.
Download the full example code here: :download:`Source <../../demo/scripts/cartesian.py>`

.. literalinclude:: ../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveRelative]
    :end-before: [initAndConfigMoveRelative]

.. _subsec-howto-stage-extension:

Stage Extension
---------------

.. _subsubsec-howto-move-relative-extension:

MoveRelative
^^^^^^^^^^^^

You may derive from this stage to extend its functionality.
``MoveRelative`` itself derives from the propagator stage that
alters solutions (i.e. computes a motion plan) when they are
passed through the stage.

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyMoveRelX

.. _subsubsec-howto-generator-extension:

Generator
^^^^^^^^^

Derive from the ``Generator`` stage to implement your own
logic in the compute function.

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyGenerator

.. _subsubsec-howto-monitoring-generator-extension:

MonitoringGenerator
^^^^^^^^^^^^^^^^^^^

Derive from the ``MonitoringGenerator`` stage to
implement your own logic in the compute function.
Use the monitoring generator instead of the normal
generator if you need to access solutions of the
monitored stage (e.g. computation of inverse kinematics).

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyMonitoringGenerator
