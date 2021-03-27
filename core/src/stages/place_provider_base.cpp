//
// Created by jafar_abdi on 3/27/21.
//

#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <moveit/robot_state/attached_body.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/stages/place_provider.h>
#include "moveit/task_constructor/stages/place_provider_base.h"
namespace moveit {
namespace task_constructor {
namespace stages {
PlaceProviderBase::PlaceProviderBase(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("object");
	p.declare<::geometry_msgs::PoseStamped_<std::allocator<void>>>("ik_frame");
}
void PlaceProviderBase::onNewSolution(const SolutionBase& s) {
	std::shared_ptr<const planning_scene::PlanningScene> scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	std::string msg;
	if (!scene->getCurrentState().hasAttachedBody(object))
		msg = "'" + object + "' is not an attached object";
	if (scene->getCurrentState().getAttachedBody(object)->getFixedTransforms().empty())
		msg = "'" + object + "' has no associated shapes";
	if (!msg.empty()) {
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else
			ROS_WARN_STREAM_NAMED("PlaceProviderBase", msg);
		return;
	}

	upstream_solutions_.push(&s);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
