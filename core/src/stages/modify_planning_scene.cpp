/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke
   Desc:    Modify planning scene
*/

#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit {
namespace task_constructor {
namespace stages {

ModifyPlanningScene::ModifyPlanningScene(const std::string& name) : PropagatingEitherWay(name) {}

void ModifyPlanningScene::attachObjects(const Names& objects, const std::string& attach_link, bool attach) {
	auto it_inserted = attach_objects_.insert(std::make_pair(attach_link, std::make_pair(Names(), attach)));
	Names& o = it_inserted.first->second.first;
	o.insert(o.end(), objects.begin(), objects.end());
}

void ModifyPlanningScene::addObject(const moveit_msgs::CollisionObject& collision_object) {
	if (collision_object.operation != moveit_msgs::CollisionObject::ADD) {
		ROS_ERROR_STREAM_NAMED("ModifyPlanningScene", name() << ": addObject is called with object's operation not set "
		                                                        "to ADD -- ignoring the object");
		return;
	}
	collision_objects_.push_back(collision_object);
}

void ModifyPlanningScene::removeObject(const std::string& object_name) {
	moveit_msgs::CollisionObject obj;
	obj.id = object_name;
	obj.operation = moveit_msgs::CollisionObject::REMOVE;
	collision_objects_.push_back(obj);
}

void ModifyPlanningScene::allowCollisions(const Names& first, const Names& second, bool allow) {
	collision_matrix_edits_.push_back(CollisionMatrixPairs({ first, second, allow }));
}

void ModifyPlanningScene::allowCollisions(const std::string& first, const moveit::core::JointModelGroup& jmg,
                                          bool allow) {
	const auto& links = jmg.getLinkModelNamesWithCollisionGeometry();
	if (!links.empty())
		allowCollisions(Names({ first }), links, allow);
}

void ModifyPlanningScene::computeForward(const InterfaceState& from) {
	sendForward(from, apply(from, false), SubTrajectory());
}

void ModifyPlanningScene::computeBackward(const InterfaceState& to) {
	sendBackward(apply(to, true), to, SubTrajectory());
}

void ModifyPlanningScene::attachObjects(planning_scene::PlanningScene& scene,
                                        const std::pair<std::string, std::pair<Names, bool> >& pair, bool invert) {
	moveit_msgs::AttachedCollisionObject obj;
	obj.link_name = pair.first;
	bool attach = pair.second.second;
	if (invert)
		attach = !attach;
	obj.object.operation =
	    attach ? (int8_t)moveit_msgs::CollisionObject::ADD : (int8_t)moveit_msgs::CollisionObject::REMOVE;
	for (const std::string& name : pair.second.first) {
		obj.object.id = name;
		scene.processAttachedCollisionObjectMsg(obj);
	}
}

void ModifyPlanningScene::allowCollisions(planning_scene::PlanningScene& scene, const CollisionMatrixPairs& pairs,
                                          bool invert) {
	collision_detection::AllowedCollisionMatrix& acm = scene.getAllowedCollisionMatrixNonConst();
	bool allow = invert ? !pairs.allow : pairs.allow;
	if (pairs.second.empty()) {
		for (const auto& name : pairs.first)
			acm.setEntry(name, allow);
	} else
		acm.setEntry(pairs.first, pairs.second, allow);
}

// invert indicates, whether to detach instead of attach (and vice versa)
// as well as to forbid instead of allow collision (and vice versa)
InterfaceState ModifyPlanningScene::apply(const InterfaceState& from, bool invert) {
	planning_scene::PlanningScenePtr scene = from.scene()->diff();
	InterfaceState result(scene);
	// add/remove objects
	for (const auto& collision_object : collision_objects_)
		processCollisionObject(*scene, collision_object);

	// attach/detach objects
	for (const auto& pair : attach_objects_)
		attachObjects(*scene, pair, invert);

	// allow/forbid collisions
	for (const auto& pairs : collision_matrix_edits_)
		allowCollisions(*scene, pairs, invert);

	if (callback_)
		callback_(scene, properties());

	return result;
}

void ModifyPlanningScene::processCollisionObject(planning_scene::PlanningScene& scene,
                                                 const moveit_msgs::CollisionObject& object) {
	scene.processCollisionObjectMsg(object);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
