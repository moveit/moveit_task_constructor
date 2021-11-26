# Migration Guide from ROS1

* Default C++ standard set to 17
* CMake version set to 3.5
* PipelinePlanner's constructor now have a node `rclcpp::Node::SharedPtr` as an argument to load the `PlanningPipeline`'s parameters
* `Task::loadRobotModel` have a node as a parameter and the user have to call loadRobotModel explicitly otherwise init will throw an exception
