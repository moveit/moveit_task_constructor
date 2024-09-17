#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages.h>

constexpr double TAU = 2 * M_PI;

using namespace moveit::task_constructor;

/** Alternatives (3x FixedState with different states) -> Fallbacks(MoveTo<CartesianPath>, MoveTo<PTP>, MoveTo<OMPL>)
 *
 * This task demonstrates how to use the Fallbacks stage to try different planning approaches in propagator.
 * Note that the initial states are all different, so this task does not describe any real-world scenario
 * (where all plans should start from the same initial state for execution).
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");

	ros::AsyncSpinner spinner{ 1 };
	spinner.start();

	// setup Task
	Task t;
	t.setName("fallback strategies in MoveTo");
	t.loadRobotModel();
	const moveit::core::RobotModelConstPtr robot{ t.getRobotModel() };

	assert(robot->getName() == "panda");

	// setup solvers
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	auto ptp = std::make_shared<solvers::PipelinePlanner>("pilz_industrial_motion_planner");
	ptp->setPlannerId("PTP");

	auto rrtconnect = std::make_shared<solvers::PipelinePlanner>("ompl");
	rrtconnect->setPlannerId("RRTConnect");

	// target end state for all Task plans
	std::map<std::string, double> target_state;
	robot->getJointModelGroup("panda_arm")->getVariableDefaultPositions("ready", target_state);
	target_state["panda_joint1"] = +TAU / 8;

	// define initial scenes
	auto initial_scene{ std::make_shared<planning_scene::PlanningScene>(robot) };
	initial_scene->getCurrentStateNonConst().setToDefaultValues(robot->getJointModelGroup("panda_arm"), "ready");

	auto initial_alternatives = std::make_unique<Alternatives>("initial states");

	{
		// can reach target with Cartesian motion
		auto fixed{ std::make_unique<stages::FixedState>("close to target state in workspace") };
		auto scene{ initial_scene->diff() };
		scene->getCurrentStateNonConst().setVariablePositions({ { "panda_joint1", -TAU / 8 } });
		fixed->setState(scene);
		initial_alternatives->add(std::move(fixed));
	}
	{
		// Cartesian motion to target is impossible, but PTP is collision-free
		auto fixed{ std::make_unique<stages::FixedState>("directly reachable without collision") };
		auto scene{ initial_scene->diff() };
		scene->getCurrentStateNonConst().setVariablePositions({
		    { "panda_joint1", +TAU / 8 },
		    { "panda_joint4", 0 },
		});
		fixed->setState(scene);
		initial_alternatives->add(std::move(fixed));
	}
	{
		// Cartesian and PTP motion to target would be in collision
		auto fixed{ std::make_unique<stages::FixedState>("getting to target requires collision avoidance") };
		auto scene{ initial_scene->diff() };
		scene->getCurrentStateNonConst().setVariablePositions({ { "panda_joint1", -TAU / 8 } });
		scene->processCollisionObjectMsg([]() {
			moveit_msgs::CollisionObject co;
			co.id = "box";
			co.header.frame_id = "panda_link0";
			co.operation = co.ADD;
			auto& pose{ co.pose };
			pose = []() {
				geometry_msgs::Pose p;
				p.position.x = 0.3;
				p.position.y = 0.0;
				p.position.z = 0.64 / 2;
				p.orientation.w = 1.0;
				return p;
			}();
			co.primitives.push_back([]() {
				shape_msgs::SolidPrimitive sp;
				sp.type = sp.BOX;
				sp.dimensions = { 0.2, 0.05, 0.64 };
				return sp;
			}());
			return co;
		}());
		fixed->setState(scene);
		initial_alternatives->add(std::move(fixed));
	}

	t.add(std::move(initial_alternatives));

	// fallbacks to reach target_state
	auto fallbacks = std::make_unique<Fallbacks>("move to other side");

	auto add_to_fallbacks{ [&](auto& solver, auto& name) {
		auto move_to = std::make_unique<stages::MoveTo>(name, solver);
		move_to->setGroup("panda_arm");
		move_to->setGoal(target_state);
		fallbacks->add(std::move(move_to));
	} };
	add_to_fallbacks(cartesian, "Cartesian path");
	add_to_fallbacks(ptp, "PTP path");
	add_to_fallbacks(rrtconnect, "RRT path");

	t.add(std::move(fallbacks));

	try {
		t.plan();
	} catch (const InitStageException& e) {
		std::cout << e << std::endl;
	}

	ros::waitForShutdown();

	return 0;
}
