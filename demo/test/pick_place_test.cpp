#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>

#include <moveit_task_constructor_demo/pick_place_task.h>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;

TEST(PickPlaceDemo, run) {
	ros::NodeHandle nh, pnh("~");

	moveit_task_constructor_demo::setupDemoScene(pnh);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);

	ASSERT_TRUE(pick_place_task.init());

	ASSERT_TRUE(pick_place_task.plan());
	ASSERT_TRUE(pick_place_task.execute());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "pick_place_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	return RUN_ALL_TESTS();
}
