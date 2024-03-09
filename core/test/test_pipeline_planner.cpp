#include "models.h"

#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace moveit::task_constructor;

// Test fixture for PipelinePlanner
struct PipelinePlannerTest : public testing::Test
{
	PipelinePlannerTest() {
		node->declare_parameter<std::vector<std::string>>("STOMP.planning_plugins", { "stomp_moveit/StompPlanner" });
	};
	const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("test_pipeline_planner");
	const moveit::core::RobotModelPtr robot_model = getModel();
};

TEST_F(PipelinePlannerTest, testInitialization) {
	// GIVEN a valid robot model, ROS node and PipelinePlanner
	auto pipeline_planner = solvers::PipelinePlanner(node, "STOMP", "stomp");
	// WHEN a PipelinePlanner instance is initialized THEN it does not throw
	EXPECT_NO_THROW(pipeline_planner.init(robot_model));
}

TEST_F(PipelinePlannerTest, testWithoutPlanningPipelines) {
	// GIVEN a PipelinePlanner instance without planning pipelines
	std::unordered_map<std::string, std::string> empty_pipeline_id_planner_id_map;
	auto pipeline_planner = solvers::PipelinePlanner(node, empty_pipeline_id_planner_id_map);
	// WHEN a PipelinePlanner instance is initialized
	// THEN it does not throw
	EXPECT_THROW(pipeline_planner.init(robot_model), std::runtime_error);
}

TEST_F(PipelinePlannerTest, testValidPlan) {
	// GIVEN an initialized PipelinePlanner
	auto pipeline_planner = solvers::PipelinePlanner(node, "STOMP", "stomp");
	pipeline_planner.init(robot_model);
	// WHEN a solution for a valid request is requested
	auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
	robot_trajectory::RobotTrajectoryPtr result =
	    std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, robot_model->getJointModelGroup("group"));
	// THEN it returns true
	EXPECT_TRUE(pipeline_planner.plan(scene, scene, robot_model->getJointModelGroup("group"), 1.0, result));
	EXPECT_EQ(pipeline_planner.getPlannerId(), "stomp");
}

TEST_F(PipelinePlannerTest, testInvalidPipelineID) {
	// GIVEN a valid initialized PipelinePlanner instance
	auto pipeline_planner = solvers::PipelinePlanner(node, "STOMP", "stomp");
	pipeline_planner.init(robot_model);
	// WHEN the planner ID for a non-existing planning pipeline is set
	// THEN setPlannerID returns false
	EXPECT_FALSE(pipeline_planner.setPlannerId("CHOMP", "stomp"));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	return RUN_ALL_TESTS();
}
