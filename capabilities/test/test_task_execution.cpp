#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit/robot_model/robot_model.hpp>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;

// provide a basic test fixture that prepares a Task
struct PandaMoveTo : public testing::Test
{
	Task t;
	stages::MoveTo* move_to;
	rclcpp::Node::SharedPtr node;

	PandaMoveTo() {
		node = rclcpp::Node::make_shared("panda_move_to");
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
	EXPECT_TRUE(t.plan());
	auto execute_result = t.execute(*t.solutions().front());
	EXPECT_TRUE(execute_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// After the arm successfully moves to "extended" try to move back to "ready" and make sure preempt() woks as expected
TEST_F(PandaMoveTo, preemptExecution) {
	move_to->setGoal("ready");
	EXPECT_TRUE(t.plan());
	// extract the expected execution time (for this task its in the last sub_trajectory)
	moveit_task_constructor_msgs::msg::Solution s;
	t.solutions().front()->toMsg(s, nullptr);
	auto exec_duration = s.sub_trajectory.back().trajectory.joint_trajectory.points.back().time_from_start;

	moveit::core::MoveItErrorCode execute_result;
	execute_result.val = execute_result.UNDEFINED;
	std::thread execute_thread{ [this, &execute_result]() { execute_result = t.execute(*t.solutions().front()); } };
	// cancel the trajectory half way through the expected execution time
	rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
	    (std::chrono::seconds(exec_duration.sec) + std::chrono::nanoseconds(exec_duration.nanosec)) / 2));
	t.preempt();
	execute_thread.join();

	EXPECT_TRUE(execute_result.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	// wait some time for move_group to come up
	rclcpp::sleep_for(std::chrono::seconds(2));

	return RUN_ALL_TESTS();
}
