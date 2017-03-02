#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/subtasks/generate_grasp_pose.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject(){
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= 0.53;
	o.primitive_poses[0].position.y= 0.55;
	o.primitive_poses[0].position.z= 0.84;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::SPHERE;
	o.primitives[0].dimensions.resize(1, 0.03);
	psi.applyCollisionObject(o);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "test_plan_current_state");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	spawnObject();

	Task t;

	auto st= std::make_shared<subtasks::GenerateGraspPose>("generate grasp candidates");

	st->setEndEffector("s_model_tool0");
	st->setGroup("arm");
	st->setObject("object");
	st->setTimeout(0.5);
	st->setAngleDelta(0.1);

	t.addStart(st);

	t.plan();

	t.printState();

	ros::spin();

	return 0;
}
