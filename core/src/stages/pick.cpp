#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/move_relative.h>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

Pick::Pick(Stage::pointer&& grasp_stage, const std::string& name)
   : SerialContainer(name)
{
	PropertyMap& p = properties();
	p.declare<std::string>("object", "name of object to grasp");
	p.declare<std::string>("eef", "end effector name");
	p.declare<std::string>("eef_frame", "name of end effector frame");

	// internal properties (cannot be marked as such yet)
	p.declare<std::string>("eef_group", "JMG of eef");
	p.declare<std::string>("eef_parent_group", "JMG of eef's parent");

	cartesian_solver_ = std::make_shared<solvers::CartesianPath>();

	{
		auto approach = std::make_unique<MoveRelative>("approach object", cartesian_solver_);
		PropertyMap& p = approach->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("link").configureInitFrom(Stage::PARENT, "eef_frame");
		p.set("marker_ns", std::string("approach"));
		approach_stage_ = approach.get();
		insert(std::move(approach));
	}

	grasp_stage_ = grasp_stage.get();
	grasp_stage->properties().configureInitFrom(Stage::PARENT, {"eef", "object"});
	insert(std::move(grasp_stage));

	{
		auto lift = std::make_unique<MoveRelative>("lift object", cartesian_solver_);
		PropertyMap& p = lift->properties();
		p.property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p.property("link").configureInitFrom(Stage::PARENT, "eef_frame");
		p.set("marker_ns", std::string("lift"));
		lift_stage_ = lift.get();
		insert(std::move(lift));
	}
}

void Pick::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	// inherit properties from parent
	PropertyMap* p = &properties();
	p->performInitFrom(Stage::PARENT, parent()->properties());

	// init internal properties
	const std::string &eef = p->get<std::string>("eef");
	const moveit::core::JointModelGroup *jmg = robot_model->getEndEffector(eef);
	p->set<std::string>("eef_group", jmg->getName());
	p->set<std::string>("eef_parent_group", jmg->getEndEffectorParentGroup().first);

	// propagate my properties to children (and do standard init)
	SerialContainer::init(robot_model);
}

void Pick::setApproachMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance)
{
	auto& p = approach_stage_->properties();
	p.set("twist", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

void Pick::setLiftMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance)
{
	auto& p = lift_stage_->properties();
	p.set("twist", motion);
	p.set("min_distance", min_distance);
	p.set("max_distance", max_distance);
}

} } }
