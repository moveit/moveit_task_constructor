
#include <moveit_task_constructor/subtasks/current_state.h>

#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>

bool
moveit::task_constructor::subtasks::CurrentState::canCompute(){
	return !succeeded_;
}

bool
moveit::task_constructor::subtasks::CurrentState::compute(){
	ros::NodeHandle h;
	ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	client.waitForExistence();

	moveit_msgs::GetPlanningScene::Request req;
	moveit_msgs::GetPlanningScene::Response res;

	req.components.components =
		moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
		| moveit_msgs::PlanningSceneComponents::ROBOT_STATE
		| moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
		| moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
		| moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
		| moveit_msgs::PlanningSceneComponents::OCTOMAP
		| moveit_msgs::PlanningSceneComponents::TRANSFORMS
		| moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
		| moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING
		| moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

	succeeded_= client.call(req, res);

	if(!succeeded_)
		return false;

	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr traj;

	planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(robot_model_));
	ps->setPlanningSceneMsg(res.scene);

	sendBothWays(traj, ps);

	return true;
}
