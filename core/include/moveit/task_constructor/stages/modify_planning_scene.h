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

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/properties.h>
#include <moveit/task_constructor/type_traits.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <map>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(JointModelGroup);
}
}  // namespace moveit

namespace moveit {
namespace task_constructor {
namespace stages {

/** Allow modification of planning scene
 *
 * This stage takes the incoming planning scene and applies previously scheduled
 * changes to it, for example:
 * - Modify allowed collision matrix, enabling or disabling collision pairs.
 * - Attach or detach objects to robot links.
 * - Spawn or remove objects.
 */
class ModifyPlanningScene : public PropagatingEitherWay
{
public:
	using Names = std::vector<std::string>;
	using ApplyCallback = std::function<void(const planning_scene::PlanningScenePtr&, const PropertyMap&)>;
	ModifyPlanningScene(const std::string& name = "modify planning scene");

	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;

	/// call an arbitrary function
	void setCallback(const ApplyCallback& cb) { callback_ = cb; }

	/// attach or detach a list of objects to the given link
	void attachObjects(const Names& objects, const std::string& attach_link, bool attach);
	/// Add an object to the planning scene
	void addObject(const moveit_msgs::msg::CollisionObject& collision_object);
	/// Remove an object from the planning scene
	void removeObject(const std::string& object_name);

	/// conviency methods accepting a single object name
	inline void attachObject(const std::string& object, const std::string& link);
	inline void detachObject(const std::string& object, const std::string& link);

	/// conviency methods accepting any container of object names
	template <typename T, typename E = typename std::enable_if_t<
	                          is_container<T>::value && std::is_base_of<std::string, typename T::value_type>::value>>
	inline void attachObjects(const T& objects, const std::string& link) {
		attachObjects(Names(objects.cbegin(), objects.cend()), link, true);
	}
	template <typename T, typename E = typename std::enable_if_t<
	                          is_container<T>::value && std::is_base_of<std::string, typename T::value_type>::value>>
	inline void detachObjects(const T& objects, const std::string& link) {
		attachObjects(Names(objects.cbegin(), objects.cend()), link, false);
	}

	/// allow / forbid collisions for each combination of pairs in first and second lists
	void allowCollisions(const Names& first, const Names& second, bool allow);
	/// allow / forbid collisions for pair (first, second)
	void allowCollisions(const std::string& first, const std::string& second, bool allow) {
		allowCollisions(Names{ first }, Names{ second }, allow);
	}
	/// allow / forbid all collisions for given object
	void allowCollisions(const std::string& object, bool allow) { allowCollisions(Names({ object }), Names(), allow); }

	/// conveniency method accepting arbitrary container types
	template <typename T1, typename T2,
	          typename E1 = typename std::enable_if_t<is_container<T1>::value &&
	                                                  std::is_convertible<typename T1::value_type, std::string>::value>,
	          typename E2 = typename std::enable_if_t<is_container<T2>::value &&
	                                                  std::is_convertible<typename T1::value_type, std::string>::value>>
	inline void allowCollisions(const T1& first, const T2& second, bool enable_collision) {
		allowCollisions(Names(first.cbegin(), first.cend()), Names(second.cbegin(), second.cend()), enable_collision);
	}
	/// conveniency method accepting std::string and an arbitrary container of names
	template <typename T, typename E = typename std::enable_if_t<
	                          is_container<T>::value && std::is_convertible<typename T::value_type, std::string>::value>>
	inline void allowCollisions(const std::string& first, const T& second, bool enable_collision) {
		allowCollisions(Names({ first }), Names(second.cbegin(), second.cend()), enable_collision);
	}
	/// conveniency method accepting const char* and an arbitrary container of names
	template <typename T, typename E = typename std::enable_if_t<
	                          is_container<T>::value && std::is_convertible<typename T::value_type, std::string>::value>>
	inline void allowCollisions(const char* first, const T& second, bool enable_collision) {
		allowCollisions(Names({ first }), Names(second.cbegin(), second.cend()), enable_collision);
	}
	/// conveniency method accepting std::string and JointModelGroup
	void allowCollisions(const std::string& first, const moveit::core::JointModelGroup& jmg, bool allow);

protected:
	// list of objects to attach (true) / detach (false) to a given link
	std::map<std::string, std::pair<Names, bool>> attach_objects_;
	// list of objects to add / remove to the planning scene
	std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;

	// list of objects to mutually
	struct CollisionMatrixPairs
	{
		Names first;
		Names second;
		bool allow;
	};
	std::list<CollisionMatrixPairs> collision_matrix_edits_;
	ApplyCallback callback_;

protected:
	// apply stored modifications to scene
	std::pair<InterfaceState, SubTrajectory> apply(const InterfaceState& from, bool invert);
	void processCollisionObject(planning_scene::PlanningScene& scene, const moveit_msgs::msg::CollisionObject& object,
	                            bool invert);
	void attachObjects(planning_scene::PlanningScene& scene, const std::pair<std::string, std::pair<Names, bool>>& pair,
	                   bool invert);
	void allowCollisions(planning_scene::PlanningScene& scene, const CollisionMatrixPairs& pairs, bool invert);
};

inline void ModifyPlanningScene::attachObject(const std::string& object, const std::string& link) {
	attachObjects(Names({ object }), link, true);
}

inline void ModifyPlanningScene::detachObject(const std::string& object, const std::string& link) {
	attachObjects(Names({ object }), link, false);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
