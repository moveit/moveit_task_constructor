#include <moveit/task_constructor/stages/fix_collision_objects.h>

#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit { namespace task_constructor { namespace stages {

FixCollisionObjects::FixCollisionObjects(std::string name)
  : PropagatingEitherWay(name)
{}

bool FixCollisionObjects::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr to = apply(from.scene(), false);
	sendForward(from, InterfaceState(to), robot_trajectory::RobotTrajectoryPtr());
	return true;
}

bool FixCollisionObjects::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from = apply(to.scene(), true);
	sendBackward(InterfaceState(from), to, robot_trajectory::RobotTrajectoryPtr());
	return true;
}

void FixCollisionObjects::setDistanceThreshold(double penetration)
{
	penetration_ = penetration;
}

void FixCollisionObjects::checkCollisions(planning_scene::PlanningScene &scene, const std::string& object)
{
	collision_detection::AllowedCollisionMatrix acm = scene.getAllowedCollisionMatrix();

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.group_name = object;
	collision_request.contacts = true;
	collision_request.max_contacts = 100;
	collision_request.max_contacts_per_pair = 5;
	collision_request.verbose = false;
	collision_request.distance = true;

	scene.checkCollision(collision_request, collision_result, scene.getCurrentState());

	geometry_msgs::Pose pose;

	if(collision_result.collision)
	{
		std::cout<<"COLLIDING"<<std::endl;
		if(collision_result.contacts.begin()->second.begin()->depth < penetration_)
		{
			pose.position.x = 0.3 + (collision_result.contacts.begin()->second.begin()->depth * collision_result.contacts.begin()->second.begin()->normal[0]);
			pose.position.y = 0.23 + (collision_result.contacts.begin()->second.begin()->depth * collision_result.contacts.begin()->second.begin()->normal[2]);
			pose.position.z = -0.02 + (collision_result.contacts.begin()->second.begin()->depth * collision_result.contacts.begin()->second.begin()->normal[1]);

			//correct collision
			moveCollisionObject(scene, pose, object, "world");
		}
	}
	else
		std::cout<<"No Collision!!!!!!!!!!!!!"<<std::endl;
}

void FixCollisionObjects::moveCollisionObject(planning_scene::PlanningScene &scene, geometry_msgs::Pose pose, const std::string& object, const std::string& base_frame)
{
	moveit_msgs::CollisionObject collision_obj;
	collision_obj.header.frame_id = base_frame;
	collision_obj.id = object;
	collision_obj.operation = moveit_msgs::CollisionObject::MOVE;

	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = pose;

	if(!scene.processCollisionObjectMsg(collision_obj))
		std::cout<<"Moving FAILED"<<std::endl;
}

planning_scene::PlanningScenePtr FixCollisionObjects::apply(const planning_scene::PlanningSceneConstPtr &scene, bool invert) const
{
	planning_scene::PlanningScenePtr result = scene->diff();

	//Check collision of objects on the table in planning scene
	(const_cast<FixCollisionObjects*>(this))->checkCollisions(*result, "object");

	return result;
}

} } }
