#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/move_relative.h>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit {
namespace task_constructor {
namespace stages {

PickPlaceBase::PickPlaceBase(Stage::pointer&& grasp_stage, const std::string& name, bool forward)
  : SerialContainer(name) {
	PropertyMap& p = properties();
	p.declare<std::string>("object", "name of object to grasp");
	p.declare<std::string>("eef", "end effector name");
	p.declare<std::string>("eef_frame", "name of end effector frame");

	// internal properties (cannot be marked as such yet)
	p.declare<std::string>("eef_group", "JMG of eef");
	p.declare<std::string>("eef_parent_group", "JMG of eef's parent");

	cartesian_solver_ = std::make_shared<solvers::CartesianPath>();
	int insertion_position = forward ? -1 : 0;  // insert children at end / front, i.e. normal or reverse order

	auto init_ik_frame = [](const PropertyMap& other) -> boost::any {
		geometry_msgs::msg::PoseStamped pose;
		const boost::any& frame = other.get("eef_frame");
		if (frame.empty())
			return boost::any();

		pose.header.frame_id = boost::any_cast<std::string>(frame);
		pose.pose.orientation.w = 1.0;
		return pose;
	};

	const auto& forwarded_props = grasp_stage->forwardedProperties();

	{
		auto approach = std::make_unique<MoveRelative>(forward ? "approach object" : "retract", cartesian_solver_);
		approach->setForwardedProperties(forwarded_props);
		PropertyMap& p = approach->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("ik_frame").configureInitFrom(Stage::PARENT, init_ik_frame);
		p.set("marker_ns", std::string(forward ? "approach" : "retract"));
		approach_stage_ = approach.get();
		insert(std::move(approach), insertion_position);
	}

	grasp_stage_ = grasp_stage.get();
	grasp_stage->properties().configureInitFrom(Stage::PARENT, { "eef", "object" });
	insert(std::move(grasp_stage), insertion_position);

	{
		auto lift = std::make_unique<MoveRelative>(forward ? "lift object" : "place object", cartesian_solver_);
		lift->setForwardedProperties(forwarded_props);
		PropertyMap& p = lift->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("ik_frame").configureInitFrom(Stage::PARENT, init_ik_frame);
		p.set("marker_ns", std::string(forward ? "lift" : "place"));
		lift_stage_ = lift.get();
		insert(std::move(lift), insertion_position);
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

	// propagate my properties to children (and do standard init)
	SerialContainer::init(robot_model);
}

void PickPlaceBase::setApproachRetract(const geometry_msgs::msg::TwistStamped& motion, double min_distance,
                                       double max_distance) {
	auto& p = approach_stage_->properties();
	p.set("direction", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setLiftPlace(const geometry_msgs::msg::TwistStamped& motion, double min_distance,
                                 double max_distance) {
	auto& p = lift_stage_->properties();
	p.set("direction", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void PickPlaceBase::setLiftPlace(const std::map<std::string, double>& joints) {
	auto& p = lift_stage_->properties();
	p.set("joints", joints);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
