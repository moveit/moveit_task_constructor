#include "models.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>

#include <moveit/planning_scene/planning_scene.hpp>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;
using namespace moveit::core;

// https://github.com/moveit/moveit_task_constructor/issues/638
TEST(SolutionMsg, DuplicateScenes) {
	Task t;
	PlanningScenePtr scene;

	t.setRobotModel(getModel());
	scene = std::make_shared<PlanningScene>(t.getRobotModel());
	t.add(std::make_unique<stages::FixedState>("start", scene));

	EXPECT_TRUE(t.plan(1));
	EXPECT_EQ(t.solutions().size(), 1u);

	// create solution
	moveit_task_constructor_msgs::msg::Solution solution_msg;
	t.solutions().front()->toMsg(solution_msg);

	// all sub trajectories `scene_diff` should be a diff
	EXPECT_EQ(solution_msg.sub_trajectory.size(), 1u);
	EXPECT_EQ(solution_msg.start_scene.is_diff, false);
	EXPECT_EQ(solution_msg.sub_trajectory.front().scene_diff.is_diff, true);
}
