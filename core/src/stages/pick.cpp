#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

Pick::Pick(Stage::pointer&& grasp_stage, const std::string& name)
   : SerialContainer(name)
{
	PropertyMap* p = &properties();
	// propagate properties from children
	p->declare<std::string>("eef", "end effector name");
	p->declare<std::string>("object", "name of object to grasp");
	p->configureInitFrom(Stage::PARENT, { "eef", "object" });

	p->declare<std::string>("eef_group", "internal");
	p->declare<std::string>("eef_parent_group", "internal");

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setTimeout(8.0);
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	{
		auto move = std::make_unique<MoveTo>("open gripper", pipeline);
		move->restrictDirection(MoveTo::FORWARD);
		p = &move->properties();
		p->property("group").configureInitFrom(Stage::PARENT, "eef_group");
		move->setGoal("open");  // TODO: retrieve from grasp stage
		insert(std::move(move));
	}

	{
		auto move = std::make_unique<Connect>("move to object", pipeline);
		p = &move->properties();
		p->property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		insert(std::move(move));
	}

	{
		auto approach = std::make_unique<MoveRelative>("approach object", cartesian);
		approach->restrictDirection(MoveRelative::BACKWARD);
		p = &approach->properties();
		p->property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p->set("marker_ns", std::string("approach"));
		approach_stage_ = approach.get();
		insert(std::move(approach));
	}

	grasp_stage_ = grasp_stage.get();
	insert(std::move(grasp_stage));

	{
		auto attach = std::make_unique<ModifyPlanningScene>("attach object");
		p = &attach->properties();
		p->configureInitFrom(Stage::PARENT, { "eef", "object" });
		attach->restrictDirection(ModifyPlanningScene::FORWARD);

		attach->setCallback([this](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
			const std::string& eef = p.get<std::string>("eef");
			moveit_msgs::AttachedCollisionObject obj;
			obj.object.operation = moveit_msgs::CollisionObject::ADD;
			obj.link_name = scene->getRobotModel()->getEndEffector(eef)->getEndEffectorParentGroup().second;
			obj.object.id = p.get<std::string>("object");
			scene->processAttachedCollisionObjectMsg(obj);
		});
		insert(std::move(attach));
	}

	{
		auto lift = std::make_unique<MoveRelative>("lift object", cartesian);
		lift->restrictDirection(MoveRelative::FORWARD);
		p = &lift->properties();
		p->property("group").configureInitFrom(Stage::PARENT, "eef_parent_group");
		p->set("marker_ns", std::string("lift"));
		lift_stage_ = lift.get();
		insert(std::move(lift));
	}
}

void Pick::init(const planning_scene::PlanningSceneConstPtr& scene)
{
	SerialContainer::init(scene);

	// init internal properties
	PropertyMap* p = &properties();
	const std::string &eef = p->get<std::string>("eef");
	const moveit::core::JointModelGroup *jmg = scene->getRobotModel()->getEndEffector(eef);
	p->set<std::string>("eef_group", jmg->getName());
	p->set<std::string>("eef_parent_group", jmg->getEndEffectorParentGroup().first);
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
