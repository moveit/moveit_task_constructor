#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/debug.h>

#include <moveit_task_constructor/stages/current_state.h>
#include <moveit_task_constructor/stages/gripper.h>
#include <moveit_task_constructor/stages/move.h>
#include <moveit_task_constructor/stages/generate_grasp_pose.h>
#include <moveit_task_constructor/stages/cartesian_position_motion.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject(){
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_link";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= 0.53;
	o.primitive_poses[0].position.y= 0.05;
	o.primitive_poses[0].position.z= 0.84;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	psi.applyCollisionObject(o);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "plan_pick");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	spawnObject();

	Task t;

	t.add( std::make_unique<stages::CurrentState>("current state") );

	{
		auto move= std::make_unique<stages::Gripper>("open gripper");
		move->setEndEffector("left_gripper");
		move->setTo("open");
		t.add(std::move(move));
	}

	{
		auto move= std::make_unique<stages::Move>("move to pre-grasp");
		move->setGroup("left_arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(std::move(move));
	}

	{
		auto move= std::make_unique<stages::CartesianPositionMotion>("proceed to grasp pose");
		move->setGroup("left_arm");
		move->setLink("l_gripper_tool_frame");
		move->setMinMaxDistance(.03, 0.1);
		move->setCartesianStepSize(0.02);

		geometry_msgs::PointStamped target;
		target.header.frame_id= "object";
		move->towards(target);
		t.add(std::move(move));
	}

	{
		auto gengrasp= std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		gengrasp->setEndEffector("left_gripper");
		//gengrasp->setGroup("arm");
		gengrasp->setLink("l_gripper_tool_frame");
		gengrasp->setGripperGraspPose("open");
		gengrasp->setObject("object");
		gengrasp->setGraspOffset(.00);
		gengrasp->setAngleDelta(.2);
		t.add(std::move(gengrasp));
	}

	{
		auto move= std::make_unique<stages::Gripper>("grasp");
		move->setEndEffector("left_gripper");
		move->setTo("closed");
		move->graspObject("object");
		t.add(std::move(move));
	}

	{
		auto move= std::make_unique<stages::CartesianPositionMotion>("lift object");
		move->setGroup("left_arm");
		move->setLink("l_gripper_tool_frame");
		move->setMinMaxDistance(0.03, 0.05);
		move->setCartesianStepSize(0.01);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id= "base_link";
		direction.vector.z= 1.0;
		move->along(direction);
		t.add(std::move(move));
	}

	t.plan();

	publishAllPlans(t);

	return 0;
}
