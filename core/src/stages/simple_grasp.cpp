/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bielefeld University
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

/* Authors: Robert Haschke */

#include <moveit/task_constructor/stages/simple_grasp.h>

#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit {
namespace task_constructor {
namespace stages {

SimpleGraspBase::SimpleGraspBase(const std::string& name) : SerialContainer(name) {
	PropertyMap& p = properties();
	p.declare<std::string>("eef", "end-effector to grasp with");
	p.declare<std::string>("object", "object to grasp");
}

// TODO: Use AttachedBody's detach_posture_ to store the inverse grasping trajectory to re-open the gripper.
void SimpleGraspBase::setup(std::unique_ptr<Stage>&& generator, bool forward) {
	// properties provided by the grasp generator via its Interface or its PropertyMap
	const std::set<std::string>& grasp_prop_names = { "object", "eef", "pregrasp", "grasp" };
	this->setForwardedProperties(grasp_prop_names);

	// insert children at end / front, i.e. normal or reverse order
	int insertion_position = forward ? -1 : (generator ? 1 : 0);

	if (generator) {
		// forward properties from generator's to IK's solution (bottom -> up)
		generator->setForwardedProperties(grasp_prop_names);
		// allow inheritance in top -> down fashion as well
		generator->properties().configureInitFrom(Stage::PARENT, { "object", "eef" });

		auto ik = new ComputeIK("compute ik", std::move(generator));
		ik->setForwardedProperties(grasp_prop_names);  // continue forwarding generator's properties

		PropertyMap& p = ik->properties();
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::INTERFACE, { "target_pose" });  // derived from child's solution
		p.configureInitFrom(Stage::PARENT, { "max_ik_solutions", "timeout", "object" });  // derived from parent
		p.configureInitFrom(Stage::PARENT | Stage::INTERFACE, { "eef", "ik_frame" });  // derive from both
		p.exposeTo(properties(), { "max_ik_solutions", "timeout", "ik_frame" });
		insert(std::unique_ptr<ComputeIK>(ik), 0);  // ComputeIK always goes upfront
	}
	{
		auto allow_touch = new ModifyPlanningScene(forward ? "allow object collision" : "forbid object collision");
		allow_touch->setForwardedProperties(grasp_prop_names);  // continue forwarding generator's properties

		PropertyMap& p = allow_touch->properties();
		p.declare<std::string>("eef");
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT | Stage::INTERFACE, { "eef", "object" });

		allow_touch->setCallback([forward](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
			collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
			const std::string& eef = p.get<std::string>("eef");
			const std::string& object = p.get<std::string>("object");
			acm.setEntry(object, scene->getRobotModel()->getEndEffector(eef)->getLinkModelNamesWithCollisionGeometry(),
			             forward);
		});
		insert(std::unique_ptr<ModifyPlanningScene>(allow_touch), insertion_position);
	}
	{
		auto planner = std::make_shared<solvers::JointInterpolationPlanner>();
		auto move = new MoveTo(forward ? "close gripper" : "open gripper", planner);
		move->setForwardedProperties(grasp_prop_names);  // continue forwarding generator's properties

		auto group_initializer = [this](const PropertyMap& parent_map) -> boost::any {
			const std::string& eef = parent_map.get<std::string>("eef");
			const moveit::core::JointModelGroup* jmg = model_->getEndEffector(eef);
			return jmg->getName();
		};
		PropertyMap& p = move->properties();
		p.property("group").configureInitFrom(Stage::PARENT | Stage::INTERFACE, group_initializer);
		p.property("goal").configureInitFrom(Stage::PARENT | Stage::INTERFACE, forward ? "grasp" : "pregrasp");
		p.exposeTo(properties(), { "group", "goal" });
		insert(std::unique_ptr<MoveTo>(move), insertion_position);
	}
	{
		auto attach = new ModifyPlanningScene(forward ? "attach object" : "detach object");
		attach->setForwardedProperties(grasp_prop_names);  // continue forwarding generator's properties

		PropertyMap& p = attach->properties();
		p.declare<std::string>("eef");
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT | Stage::INTERFACE, { "eef", "object" });

		attach->setCallback([forward](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
			const std::string& eef = p.get<std::string>("eef");
			moveit_msgs::msg::AttachedCollisionObject obj;
			obj.object.operation = forward ? static_cast<int8_t>(moveit_msgs::msg::CollisionObject::ADD) :
			                                 static_cast<int8_t>(moveit_msgs::msg::CollisionObject::REMOVE);
			obj.link_name = scene->getRobotModel()->getEndEffector(eef)->getEndEffectorParentGroup().second;
			obj.object.id = p.get<std::string>("object");
			scene->processAttachedCollisionObjectMsg(obj);
		});
		insert(std::unique_ptr<ModifyPlanningScene>(attach), insertion_position);
	}
}

void SimpleGraspBase::init(const moveit::core::RobotModelConstPtr& robot_model) {
	model_ = robot_model;
	SerialContainer::init(robot_model);
}

void SimpleGraspBase::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	pose_msg.pose = tf2::toMsg(pose);
	setIKFrame(pose_msg);
}

SimpleGrasp::SimpleGrasp(std::unique_ptr<Stage>&& generator, const std::string& name) : SimpleGraspBase(name) {
	setup(std::move(generator), true);
}

SimpleUnGrasp::SimpleUnGrasp(std::unique_ptr<Stage>&& generator, const std::string& name) : SimpleGraspBase(name) {
	setup(std::move(generator), false);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
