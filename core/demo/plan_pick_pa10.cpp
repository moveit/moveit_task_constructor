#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>

#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

void spawnObject(planning_scene::PlanningScenePtr scene) {
	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= 0.3;
	o.primitive_poses[0].position.y= 0.23;
	o.primitive_poses[0].position.z= 0.12;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	scene->processCollisionObjectMsg(o);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "plan_pick");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;
	// define global properties used by most stages
	t.setProperty("group", std::string("left_arm"));
	t.setProperty("eef", std::string("la_tool_mount"));
	t.setProperty("gripper", std::string("left_hand"));
	t.setProperty("link", std::string("la_tool_mount"));

	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setTimeout(8.0);
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	t.add(std::make_unique<stages::CurrentState>());
	{
		auto stage = std::make_unique<stages::FixCollisionObjects>();
		stage->restrictDirection(stages::MoveTo::FORWARD);
		stage->setMaxPenetration(0.1);
		t.add(std::move(stage));
	}
	{
		auto move = std::make_unique<stages::MoveTo>("open gripper", pipeline);
		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		move->setGoal("open");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::Connect>("move to object", pipeline);
		move->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("approach object", cartesian);
		move->restrictDirection(stages::MoveRelative::BACKWARD);
		move->properties().configureInitFrom(Stage::PARENT);
		move->properties().set("marker_ns", std::string("approach"));
		move->properties().set("link", std::string("lh_tool_frame"));
		move->setMinMaxDistance(0.05, 0.1);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "lh_tool_frame";
		direction.vector.z = 1;
		move->along(direction);
		t.add(std::move(move));
	}

	{
		auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		gengrasp->properties().configureInitFrom(Stage::PARENT);
		gengrasp->setNamedPose("open");
		gengrasp->setObject("object");
		gengrasp->setToolToGraspTF(Eigen::Translation3d(0,0,.05)*
		                           Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY()),
		                           "lh_tool_frame");
		gengrasp->setAngleDelta(M_PI / 10.);

		auto ik = std::make_unique<stages::ComputeIK>("compute ik", std::move(gengrasp));
		ik->properties().configureInitFrom(Stage::PARENT, {"group", "eef", "default_pose"});
		ik->setMaxIKSolutions(1);
		t.add(std::move(ik));
	}

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("enable object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->enableCollisions("object", t.getRobotModel()->getJointModelGroup("left_hand")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveTo>("close gripper", pipeline);
		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		move->setGoal("closed");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("attach object");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);
		move->attachObject("object", "lh_tool_frame");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("lift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, {"group"});
		move->setMinMaxDistance(0.03, 0.05);
		move->properties().set("marker_ns", std::string("lift"));
		move->properties().set("link", std::string("lh_tool_frame"));

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 1;
		move->along(direction);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("shift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, {"group"});
		move->setMinMaxDistance(0.1, 0.2);
		move->properties().set("marker_ns", std::string("lift"));
		move->properties().set("link", std::string("lh_tool_frame"));

		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "object";
		twist.twist.linear.y = 1;
		twist.twist.angular.y = 2;
		move->along(twist);
		t.add(std::move(move));
	}

	try {
		auto scene = t.initScene(ros::Duration(0));
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup("left_arm"), "home");
		state.setToDefaultValues(state.getJointModelGroup("right_arm"), "home");
		state.update();
		spawnObject(scene);
		t.plan();
		std::cout << "waiting for <enter>\n";
		char ch;
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		t.printState();
		return EINVAL;
	}

	return 0;
}
