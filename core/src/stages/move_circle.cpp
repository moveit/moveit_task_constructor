// TODO: Add doc

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/task_constructor/stages/move_circle.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/moveit_compat.h>
#include <moveit/task_constructor/utils.h>

#include <rviz_marker_tools/marker_creation.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/robot_state/conversions.h>

namespace moveit::task_constructor::stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MoveCircle");

MoveCircle::MoveCircle(const std::string& name)
  : PropagatingEitherWay(name) {
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	p.property("timeout").setDefaultValue(1.0);
	p.declare<std::string>("group", "name of planning group");
	p.declare<boost::any>("ik_frame", "name of IK frame");
	p.declare<geometry_msgs::msg::PoseStamped>("goal", "goal pose specification");
	p.declare<std::pair<std::string, geometry_msgs::msg::PoseStamped>>("arc_constraint", "arc constraint specification");
	// register actual types
	PropertySerializer<geometry_msgs::msg::PoseStamped>();
}

void MoveCircle::init(const moveit::core::RobotModelConstPtr& robot_model) {
	PropagatingEitherWay::init(robot_model);

	// Create a Pilz circular trajectory generator
	using pilz_industrial_motion_planner::TrajectoryGeneratorCIRC;
	using pilz_industrial_motion_planner::CartesianLimit;
	using pilz_industrial_motion_planner::LimitsContainer;

	// TODO: Get these limits from somewhere else,
	// and also set joint limits (from robot model?)
	auto cart_limits = CartesianLimit();
	cart_limits.setMaxRotationalVelocity(1.0);
	cart_limits.setMaxTranslationalVelocity(1.0);
	cart_limits.setMaxTranslationalAcceleration(1.0);
	cart_limits.setMaxTranslationalDeceleration(-1.0);
	auto limits = LimitsContainer();
	limits.setCartesianLimits(cart_limits);

	const auto group = properties().get<std::string>("group");
	planner_ = std::make_shared<TrajectoryGeneratorCIRC>(robot_model, limits, group);
}

bool MoveCircle::compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& solution,
                         Interface::Direction dir) {
	scene = state.scene()->diff();
	const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
	assert(robot_model);
	const auto model_frame = robot_model->getModelFrame();

	const auto& props = properties();
	double timeout = this->timeout();
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
	if (!jmg) {
		solution.markAsFailure("invalid joint model group: " + group);
		return false;
	}
	const auto& goal = props.get<geometry_msgs::msg::PoseStamped>("goal");
	const auto& arc_constraint = props.get<std::pair<std::string, geometry_msgs::msg::PoseStamped>>("arc_constraint");
	if (arc_constraint.first != "center" && arc_constraint.first != "interim") {
		solution.markAsFailure("invalid arc constraint: " + arc_constraint.first + 
		                       ". Must be either center or interim.");
		return false;	
	}
	auto link = properties().get<std::string>("link");
	RCLCPP_INFO(LOGGER, "Properties validated!");

	////////////////////
	// Convert frames //
	////////////////////
	geometry_msgs::msg::PoseStamped link_frame;
	const moveit::core::LinkModel* link_model;
	link_frame.header.frame_id = properties().get<std::string>("link");
	link_frame.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
	setProperty("ik_frame", link_frame);

	Eigen::Isometry3d ik_pose_world;
	if (!utils::getRobotTipForFrame(props.property("ik_frame"), *scene, jmg, solution, link_model, ik_pose_world)) {
		return false;
	}
	auto ik_link_name = link_model->getName();
	RCLCPP_INFO(LOGGER, "Converted link to %s", ik_link_name.c_str());
	RCLCPP_INFO(LOGGER, "IK POSE WORLD: X=%f Y=%f Z=%f",
		ik_pose_world.translation().x(), ik_pose_world.translation().y(), ik_pose_world.translation().z());
	auto ik_frame_to_tip = ik_pose_world.inverse() * scene->getCurrentState().getGlobalLinkTransform(link_model);
	RCLCPP_INFO(LOGGER, "IK FRAME TO TIP: X=%f Y=%f Z=%f",
		ik_frame_to_tip.translation().x(), ik_frame_to_tip.translation().y(), ik_frame_to_tip.translation().z());
	
	// transform target into global frame
	Eigen::Isometry3d target;
	tf2::fromMsg(goal.pose, target);
	target = scene->getFrameTransform(goal.header.frame_id) * target * ik_frame_to_tip;
	geometry_msgs::msg::PoseStamped new_goal;
	new_goal.header.frame_id = ik_link_name;
	new_goal.pose = Eigen::toMsg(target);
	// new_goal = goal;
	RCLCPP_INFO(LOGGER, "Goal pose in link frame: X=%f Y=%f Z=%f",
		new_goal.pose.position.x, new_goal.pose.position.y, new_goal.pose.position.z);

	// transform constraint position into global frame
	Eigen::Isometry3d arc_target;
	tf2::fromMsg(arc_constraint.second.pose, arc_target);
	arc_target = scene->getFrameTransform(arc_constraint.second.header.frame_id) * arc_target * ik_frame_to_tip;
	geometry_msgs::msg::PoseStamped new_arc_constraint_pose;
	new_arc_constraint_pose.header.frame_id = ik_link_name;
	new_arc_constraint_pose.pose = Eigen::toMsg(arc_target);
	RCLCPP_INFO(LOGGER, "Constraint pose in link frame: X=%f Y=%f Z=%f", new_arc_constraint_pose.pose.position.x,
	            new_arc_constraint_pose.pose.position.y, new_arc_constraint_pose.pose.position.z);


	//////////
	// PLAN //
	//////////
	RCLCPP_INFO(LOGGER, "Planning!");
	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;

	// Package up a motion plan request for Pilz
	planning_interface::MotionPlanRequest motion_request;
	motion_request.pipeline_id = "pilz_industrial_motion_planner";
	motion_request.planner_id = "CIRC";
	motion_request.group_name = group;
	motion_request.max_velocity_scaling_factor = 1.0;  // TODO: Get from props
	motion_request.max_acceleration_scaling_factor = 1.0;  // TODO: Get from props
	moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), motion_request.start_state);
	
	// Add goal constraint for target pose
	motion_request.goal_constraints.resize(1);
	motion_request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(ik_link_name, new_goal);

	// Add path constraints for the interim / center
	motion_request.path_constraints.name = arc_constraint.first;
	moveit_msgs::msg::PositionConstraint pos_constraint;
	pos_constraint.header.frame_id = new_arc_constraint_pose.header.frame_id;
	pos_constraint.link_name = ik_link_name;
	pos_constraint.constraint_region.primitive_poses.push_back(new_arc_constraint_pose.pose);
	pos_constraint.weight = 1.0;  // Shouldn't matter as it's the only constraint
	motion_request.path_constraints.position_constraints.resize(1);
	motion_request.path_constraints.position_constraints[0] = pos_constraint;

	planning_interface::MotionPlanResponse motion_response;
	const double sampling_time = 0.001;  // TODO Get from props
	
	success = planner_->generate(scene, motion_request, motion_response, sampling_time);
	RCLCPP_INFO(LOGGER, "Got trajectory with %lu waypoints",
		motion_response.trajectory_->getWayPointCount());

	// store result
	robot_trajectory = motion_response.trajectory_;
	if (!robot_trajectory && storeFailures()) {
		robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, jmg);
		robot_trajectory->addSuffixWayPoint(state.scene()->getCurrentState(), 0.0);
		robot_trajectory->addSuffixWayPoint(scene->getCurrentState(), 1.0);
	}
	if (robot_trajectory) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == Interface::BACKWARD)
			robot_trajectory->reverse();
		solution.setTrajectory(robot_trajectory);

		if (!success)
			solution.markAsFailure();

		return true;
	}
	return false;
}

}  // namespace moveit::task_constructor::stages
