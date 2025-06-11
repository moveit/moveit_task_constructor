#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit/robot_model/robot_model.h>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;

// provide a basic test fixture that prepares a Task
struct PandaMoveTo : public testing::Test
{
	Task t;
	stages::MoveTo* move_to;
	rclcpp::Node::SharedPtr node;

	PandaMoveTo() {
		node = rclcpp::Node::make_shared("test_task_execution");
		t.loadRobotModel(node);

		auto group = t.getRobotModel()->getJointModelGroup("panda_arm");

		auto initial = std::make_unique<stages::CurrentState>("current state");
		t.add(std::move(initial));

		auto move = std::make_unique<stages::MoveTo>("move", std::make_shared<solvers::JointInterpolationPlanner>());
		move_to = move.get();
		move_to->setGroup(group->getName());
		t.add(std::move(move));
	}
};

// The arm starts in the "ready" pose so make sure we can move to a different known location
TEST_F(PandaMoveTo, successExecution) {
	move_to->setGoal("extended");
	ASSERT_TRUE(t.plan());
	auto result = t.execute(*t.solutions().front());
	EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// After the arm successfully moved to "extended", move back to "ready" and make sure preempt() works as expected
TEST_F(PandaMoveTo, preemptExecution) {
	move_to->setGoal("ready");
	ASSERT_TRUE(t.plan());
	// extract the expected execution time (for this task its in the last sub_trajectory)
	moveit_task_constructor_msgs::msg::Solution s;
	t.solutions().front()->toMsg(s, nullptr);
	rclcpp::Duration exec_duration{ s.sub_trajectory.back().trajectory.joint_trajectory.points.back().time_from_start };

	moveit::core::MoveItErrorCode result;
	std::thread execute_thread{ [this, &result]() { result = t.execute(*t.solutions().front()); } };

	// cancel the trajectory half way through the expected execution time
	rclcpp::sleep_for(exec_duration.to_chrono<std::chrono::milliseconds>() / 2);
	t.preempt();
	if (execute_thread.joinable()) {
		execute_thread.join();
	}

	EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);

	// After preempting motion reset the task and make sure we can successfully plan and execute motion again
	rclcpp::sleep_for(std::chrono::seconds(1));
	t.reset();
	ASSERT_TRUE(t.plan());
	result = t.execute(*t.solutions().front());
	EXPECT_EQ(result.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	// wait some time for move_group to come up
	rclcpp::sleep_for(std::chrono::seconds(5));

	return RUN_ALL_TESTS();
}
