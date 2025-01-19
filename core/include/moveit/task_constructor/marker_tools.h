#pragma once

#include <rviz_marker_tools/marker_creation.h>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/macros/class_forward.h>
#include <functional>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}
namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
MOVEIT_CLASS_FORWARD(LinkModel);
}  // namespace core
}  // namespace moveit

namespace moveit {
namespace task_constructor {

/** signature of callback function, passing the generated marker and the name of the robot link / scene object */
using MarkerCallback = std::function<void(visualization_msgs::msg::Marker&, const std::string&)>;

/** generate marker msgs to visualize the planning scene, calling the given callback for each of them
 *  object_names: set of links to include (or all if empty) */
void generateMarkersForObjects(const planning_scene::PlanningSceneConstPtr& scene, const MarkerCallback& callback,
                               const std::vector<std::string>& object_names = {});

/** generate marker msgs to visualize robot's collision geometry, calling the given callback for each of them
 *  link_names: set of links to include (or all if empty) */
void generateCollisionMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                              const std::vector<std::string>& link_names = {});
void generateCollisionMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                              const std::vector<const moveit::core::LinkModel*>& link_models);

/** generate marker msgs to visualize robot's visual geometry, calling the given callback for each of them
 *  link_names: set of links to include (or all if empty) */
void generateVisualMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                           const std::vector<std::string>& link_names = {});
void generateVisualMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                           const std::vector<const moveit::core::LinkModel*>& link_models);

/** generate marker msgs to visualize the planning scene, calling the given callback for each of them
 *  calls generateMarkersForRobot() and generateMarkersForObjects() */
void generateMarkersForScene(const planning_scene::PlanningSceneConstPtr& scene, const MarkerCallback& callback);
}  // namespace task_constructor
}  // namespace moveit
