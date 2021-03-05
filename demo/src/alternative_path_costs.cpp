#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>

#include <moveit/task_constructor/cost_terms.h>

using namespace moveit::task_constructor;

/* FixedState - Connect - FixedState */
int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");

	Task t;
	t.stages()->setName("alternative path costs");
	t.loadRobotModel();

	assert(t.getRobotModel()->getName() == "panda");

	auto scene{ std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };
	auto& robot_state{ scene->getCurrentStateNonConst() };
	robot_state.setToDefaultValues();
	robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "extended");

	auto initial{ std::make_unique<stages::FixedState>("start") };
	initial->setState(scene);
	t.add(std::move(initial));

	auto pipeline{ std::make_shared<solvers::PipelinePlanner>() };

	auto alternatives{ std::make_unique<Alternatives>("connect") };
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "path length", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::PathLength>());  // This is the default for Connect, specified for
		                                                             // demonstration purposes
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "trajectory duration", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "eef motion", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::LinkMotion>("panda_hand"));
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "elbow motion", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::LinkMotion>("panda_link4"));
		alternatives->add(std::move(connect));
	}

	t.add(std::move(alternatives));

	auto goal_scene{ scene->diff() };
	goal_scene->getCurrentStateNonConst().setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "ready");
	auto goal = std::make_unique<stages::FixedState>("goal");
	goal->setState(goal_scene);
	t.add(std::move(goal));

	try {
		t.plan(0);
	} catch (const InitStageException& e) {
		std::cout << e << std::endl;
	}

	ros::spin();

	return 0;
}
