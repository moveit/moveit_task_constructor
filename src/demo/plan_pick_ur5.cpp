#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/debug.h>

#include <moveit_task_constructor/subtasks/current_state.h>
#include <moveit_task_constructor/subtasks/gripper.h>
#include <moveit_task_constructor/subtasks/move.h>
#include <moveit_task_constructor/subtasks/generate_grasp_pose.h>
#include <moveit_task_constructor/subtasks/cartesian_position_motion.h>

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject(){
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "table_top";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= -0.2;
	o.primitive_poses[0].position.y= 0.13;
	o.primitive_poses[0].position.z= 0.12;
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

	t.add( std::make_shared<subtasks::CurrentState>("current state") );

	{
		auto move= std::make_shared<subtasks::Gripper>("open gripper");
		move->setEndEffector("gripper");
		move->setTo("open");
		t.add(move);
	}

	{
		auto move= std::make_shared<subtasks::Move>("move to pre-grasp");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(move);
	}

	{
		auto move= std::make_shared<subtasks::CartesianPositionMotion>("proceed to grasp pose");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(.03, 0.1);
		move->setCartesianStepSize(0.02);

		geometry_msgs::PointStamped target;
		target.header.frame_id= "object";
		move->towards(target);
		t.add(move);
	}

	{
		auto gengrasp= std::make_shared<subtasks::GenerateGraspPose>("generate grasp pose");
		gengrasp->setEndEffector("gripper");
		//gengrasp->setGroup("arm");
		gengrasp->setGripperGraspPose("open");
		gengrasp->setObject("object");
		gengrasp->setGraspOffset(.03);
		gengrasp->setAngleDelta(-.2);
		gengrasp->setMaxIKSolutions(8);
		t.add(gengrasp);
	}

	{
		auto move= std::make_shared<subtasks::Gripper>("grasp");
		move->setEndEffector("gripper");
		move->setTo("closed");
		move->graspObject("object");
		t.add(move);
	}

	{
		auto move= std::make_shared<subtasks::CartesianPositionMotion>("lift object");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(0.03, 0.05);
		move->setCartesianStepSize(0.01);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id= "world";
		direction.vector.z= 1.0;
		move->along(direction);
		t.add(move);
	}

	t.plan();

	publishAllPlans(t);

	return 0;
}
