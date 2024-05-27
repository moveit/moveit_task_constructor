#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/cost_terms.h>
#include "ik_clearance_cost_parameters.hpp"

using namespace moveit::task_constructor;

/* ComputeIK(FixedState) */
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("moveit_task_constructor_demo");
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	const auto param_listener = std::make_shared<ik_clearance_cost_demo::ParamListener>(node);
	const auto params = param_listener->get_params();

	Task t;
	t.stages()->setName("clearance IK");

	// announce new task (in case previous run was restarted)
	rclcpp::sleep_for(std::chrono::milliseconds(500));

	t.loadRobotModel(node);
	assert(t.getRobotModel()->getName() == "panda");

	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	auto& robot_state = scene->getCurrentStateNonConst();
	robot_state.setToDefaultValues();
	[[maybe_unused]] bool found =
	    robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "extended");
	assert(found);

	moveit_msgs::msg::CollisionObject co;
	co.id = "obstacle";
	co.primitives.emplace_back();
	co.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
	co.primitives[0].dimensions.resize(1);
	co.primitives[0].dimensions[0] = 0.1;
	co.header.frame_id = t.getRobotModel()->getModelFrame();
	co.primitive_poses.emplace_back();
	co.primitive_poses[0].orientation.w = 1.0;
	co.primitive_poses[0].position.z = 0.85;
	scene->processCollisionObjectMsg(co);

	auto initial = std::make_unique<stages::FixedState>();
	initial->setState(scene);
	initial->setIgnoreCollisions(true);

	auto ik = std::make_unique<stages::ComputeIK>();
	ik->insert(std::move(initial));
	ik->setGroup("panda_arm");
	ik->setTargetPose(Eigen::Translation3d(.3, 0, .35) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
	ik->setTimeout(1.0);
	ik->setMaxIKSolutions(100);

	auto cl_cost{ std::make_unique<cost::Clearance>() };
	cl_cost->cumulative = params.cumulative;
	cl_cost->with_world = params.with_world;
	ik->setCostTerm(std::move(cl_cost));

	t.add(std::move(ik));

	try {
		t.plan(0);
	} catch (const InitStageException& e) {
		std::cout << e << std::endl;
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}
