#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/grasp_provider.h>
#include <moveit/task_constructor/stages/place_provider.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/planning_scene/planning_scene.h>

#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace stages {

PickPlaceBase::PickPlaceBase(const std::string& name, const std::string& provider_stage_plugin_name,  bool is_pick)
  : SerialContainer(name),
  grasp_provider_class_loader_("moveit_task_constructor_core", "moveit::task_constructor::stages::GraspProviderBase"),
  place_provider_class_loader_("moveit_task_constructor_core", "moveit::task_constructor::stages::PlaceProviderBase") {
	PropertyMap& p = properties();
	p.declare<std::string>("object", "name of object to grasp/place");
	p.declare<std::string>("eef", "end effector name");
	p.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose");
	p.declare<std::string>("eef_group_open_pose", "Name of open pose of eef_group");
	p.declare<std::string>("eef_group_close_pose", "Name of close pose of eef_group");
	p.declare<std::vector<std::string>>("support_surfaces", {}, "Name of support surfaces");

	// internal properties (cannot be marked as such yet)
	p.declare<std::string>("eef_group", "JMG of eef");
	p.declare<std::string>("eef_parent_group", "JMG of eef's parent");

	cartesian_solver_ = std::make_shared<solvers::CartesianPath>();
	sampling_planner_ = std::make_shared<solvers::PipelinePlanner>();

	is_pick_ = is_pick;

	provider_stage_plugin_name_ = provider_stage_plugin_name;

	{
		auto move_there = std::make_unique<MoveRelative>(is_pick_ ? "approach object" : "lower object", cartesian_solver_);
		PropertyMap& p = move_there->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("ik_frame").configureInitFrom(Stage::PARENT, "ik_frame");
		p.set("marker_ns", std::string(is_pick_ ? "approach" : "place"));
		move_there_stage_ = move_there.get();
		insert(std::move(move_there));
	}

	if (provider_stage_plugin_name_ == "")
	{
		if (is_pick_)
			provider_stage_plugin_name_ = "moveit_task_constructor/GraspProviderDefault";
		else
			provider_stage_plugin_name_ = "moveit_task_constructor/PlaceProviderDefault";
		ROS_WARN_STREAM("The given name of the provider stage plugin is an empty string, using the default plugin (" << provider_stage_plugin_name_ << ") instead!");
	}

	try
	{
		std::unique_ptr<stages::ComputeIK> wrapper;
		if (is_pick_) {
			std::unique_ptr<GraspProviderBase> provider_stage_plugin(grasp_provider_class_loader_.createUnmanagedInstance(provider_stage_plugin_name_));
			provider_stage_plugin->properties().configureInitFrom(Stage::PARENT);
			provider_stage_plugin->properties().property("pregrasp").configureInitFrom(Stage::PARENT, "eef_group_open_pose");
			provider_stage_plugin->properties().property("grasp").configureInitFrom(Stage::PARENT, "eef_group_close_pose");
			provider_stage_plugin->properties().set("marker_ns", "grasp_pose");
			grasp_stage_ = provider_stage_plugin.get();

			wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(provider_stage_plugin));
		}
		else {
			std::unique_ptr<PlaceProviderBase> provider_stage_plugin(place_provider_class_loader_.createUnmanagedInstance(provider_stage_plugin_name_));
			provider_stage_plugin->properties().configureInitFrom(Stage::PARENT);
			provider_stage_plugin->properties().set("marker_ns", "place_pose");
			place_stage_ = provider_stage_plugin.get();

			wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(provider_stage_plugin));
		}

		properties().exposeTo(wrapper->properties(), {"object", "eef_group_open_pose", "eef_group_close_pose"});
		wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame", "object", "eef_group_open_pose", "eef_group_close_pose"});
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		compute_ik_stage_ = wrapper.get();
		insert(std::move(wrapper));

	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The provider stage plugin failed to load. Error: %s", ex.what());
	}

	{
		auto set_collision_object_hand = std::make_unique<stages::ModifyPlanningScene>(is_pick_ ? "allow collision (hand,object)" : "forbid collision (hand,object)");
		set_collision_object_hand_stage_ = set_collision_object_hand.get();
		insert(std::move(set_collision_object_hand));
	}

	{
		auto open_close_hand = std::make_unique<stages::MoveTo>(is_pick_ ? "close hand" : "open hand", sampling_planner_);
		PropertyMap& p = open_close_hand->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_group");
		p.property("goal").configureInitFrom(Stage::PARENT, is_pick_ ? "eef_group_close_pose" : "eef_group_open_pose");
		if (is_pick_)
			insert(std::move(open_close_hand));
		else
			insert(std::move(open_close_hand), -2);
	}

	{
		auto attach_detach = std::make_unique<stages::ModifyPlanningScene>(is_pick_ ? "attach object" : "detach object");
		attach_detach_stage_ = attach_detach.get();
		insert(std::move(attach_detach));
	}

	if (is_pick_) {
		auto set_collision_object_support = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
		allow_collision_object_support_stage_ = set_collision_object_support.get();
		insert(std::move(set_collision_object_support));
	}

	{
		auto move_back = std::make_unique<MoveRelative>(is_pick_ ? "lift object" : "retract", cartesian_solver_);
		PropertyMap& p = move_back->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("ik_frame").configureInitFrom(Stage::PARENT, "ik_frame");
		p.set("marker_ns", std::string(is_pick_ ? "lift" : "retract"));
		move_back_stage_ = move_back.get();
		insert(std::move(move_back));
	}

	if (is_pick_) {
		auto set_collision_object_support = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
		forbid_collision_object_support_stage_ = set_collision_object_support.get();
		insert(std::move(set_collision_object_support));
	}
}

void PickPlaceBase::init(const moveit::core::RobotModelConstPtr& robot_model) {
	// inherit properties from parent
	PropertyMap* p = &properties();
	p->performInitFrom(Stage::PARENT, parent()->properties());

	// init internal properties
	const std::string& eef = p->get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
	if (!jmg)
		throw InitStageException(*this, "unknown end effector: " + eef);

	p->set<std::string>("eef_group", jmg->getName());
	p->set<std::string>("eef_parent_group", jmg->getEndEffectorParentGroup().first);

	set_collision_object_hand_stage_->allowCollisions(properties().get<std::string>("object"),
		jmg->getLinkModelNamesWithCollisionGeometry(), is_pick_);

	const std::string& object = properties().get<std::string>("object");
	const geometry_msgs::PoseStamped& ik_frame = properties().get<geometry_msgs::PoseStamped>("ik_frame");
	const std::vector<std::string>& support_surfaces = properties().get<std::vector<std::string>>("support_surfaces");
	if (is_pick_) {
		attach_detach_stage_->attachObject(object, ik_frame.header.frame_id);
		allow_collision_object_support_stage_->allowCollisions({ object }, support_surfaces, true);
		forbid_collision_object_support_stage_->allowCollisions({ object }, support_surfaces, false);
	}
	else
		attach_detach_stage_->detachObject(object, ik_frame.header.frame_id);

	// propagate my properties to children (and do standard init)
	SerialContainer::init(robot_model);
}


/// -------------------------
/// setters of own properties

void PickPlaceBase::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setIKFrame(pose_msg);
}


/// -------------------------
/// setters of substage properties


/// approach / place

void PickPlaceBase::setApproachPlace(const geometry_msgs::TwistStamped& motion, double min_distance,
                                       double max_distance) {
	auto& p = move_there_stage_->properties();
	p.set("direction", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setApproachPlace(const geometry_msgs::Vector3Stamped& direction, double min_distance,
                                       double max_distance) {
	auto& p = move_there_stage_->properties();
	p.set("direction", direction);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setApproachPlace(const std::map<std::string, double>& joints) {
	auto& p = move_there_stage_->properties();
	p.set("joints", joints);
}

/// IK computation

void PickPlaceBase::setMaxIKSolutions(const uint32_t& max_ik_solutions) {
	auto& p = compute_ik_stage_->properties();
	p.set("max_ik_solutions", max_ik_solutions);
}

void PickPlaceBase::setMinIKSolutionDistance(const double& min_ik_solution_distance) {
	auto& p = compute_ik_stage_->properties();
	p.set("min_solution_distance", min_ik_solution_distance);
}

void PickPlaceBase::setIgnoreIKCollisions(const bool& ignore_ik_collisions) {
	auto& p = compute_ik_stage_->properties();
	p.set("ignore_collisions", ignore_ik_collisions);
}


/// lift / retract

void PickPlaceBase::setLiftRetract(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance) {
	auto& p = move_back_stage_->properties();
	p.set("direction", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setLiftRetract(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance) {
	auto& p = move_back_stage_->properties();
	p.set("direction", direction);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setLiftRetract(const std::map<std::string, double>& joints) {
	auto& p = move_back_stage_->properties();
	p.set("joints", joints);
}


/// -------------------------
/// setters of pick properties

void Pick::setMonitoredStage(Stage* monitored) {
	grasp_stage_->setMonitoredStage(monitored);
}


/// -------------------------
/// setters of place properties

void Place::setMonitoredStage(Stage* monitored) {
	place_stage_->setMonitoredStage(monitored);
}

void Place::setPlacePose(const geometry_msgs::PoseStamped& pose) {
	place_stage_->setPose(pose);
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
