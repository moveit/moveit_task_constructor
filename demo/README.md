# moveit_task_constructor_demo

Demos illustrating the capabilities of MoveIt Task Constructor.
All demos use the Panda robot arm from Franka Emika.

## Run

Prepare: Launch MoveIt and rviz with the Panda robot

    ros2 launch moveit_task_constructor_demo demo.launch.py

Run a specific demo, e.g., the pick-and-place demo or the Cartesian demo:

    ros2 launch moveit_task_constructor_demo run.launch.py exe:=pick_place_demo
    ros2 launch moveit_task_constructor_demo run.launch.py exe:=cartesian.py
