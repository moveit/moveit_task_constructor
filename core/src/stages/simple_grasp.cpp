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
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

SimpleGraspBase::SimpleGraspBase(const std::string& name)
   : SerialContainer(name)
{
	auto &props = properties();
	props.declare<boost::any>("pregrasp", "pregrasp posture");
	props.declare<boost::any>("grasp", "grasp posture");
}

void SimpleGraspBase::setup(std::unique_ptr<MonitoringGenerator>&& generator, bool forward)
{
	int insertion_position = forward ? -1 : 0; // insert children at end / front, i.e. normal or reverse order
	{
		generator_ = generator.get();
		auto ik = new ComputeIK("compute ik", std::move(generator));
		const std::initializer_list<std::string>& grasp_prop_names = { "eef", "pregrasp", "object" };
		ik->exposePropertiesOfChild(0, grasp_prop_names, true);
		insert(std::unique_ptr<ComputeIK>(ik), insertion_position);

		exposePropertiesOfChild(insertion_position, grasp_prop_names, true);
		exposePropertiesOfChild(insertion_position, { "max_ik_solutions", "timeout", "ik_frame" });
	}
	{
		auto allow_touch = new ModifyPlanningScene(forward ? "allow object collision" : "forbid object collision");
		PropertyMap& p = allow_touch->properties();
		p.declare<std::string>("eef");
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT, { "eef", "object" });

		allow_touch->setCallback([this](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
			collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
			const std::string& eef = p.get<std::string>("eef");
			const std::string& object = p.get<std::string>("object");
			acm.setEntry(object, scene->getRobotModel()->getEndEffector(eef)
			             ->getLinkModelNamesWithCollisionGeometry(), true);
		});
		insert(std::unique_ptr<ModifyPlanningScene>(allow_touch), insertion_position);
	}
	{
		auto pipeline = std::make_shared<solvers::PipelinePlanner>();
		pipeline->setTimeout(8.0);
		pipeline->setPlannerId("RRTConnectkConfigDefault");

		auto move = new MoveTo(forward ? "close gripper" : "open gripper", pipeline);
		PropertyMap& p = move->properties();
		p.property("group").configureInitFrom(Stage::PARENT, [this](const PropertyMap& parent_map){
			const std::string& eef = parent_map.get<std::string>("eef");
			const moveit::core::JointModelGroup* jmg = model_->getEndEffector(eef);
			return boost::any(jmg->getName());
		});
		insert(std::unique_ptr<MoveTo>(move), insertion_position);
		exposePropertyOfChildAs(insertion_position, "goal", forward ? "grasp" : "pregrasp");
	}
	{
		auto attach = new ModifyPlanningScene(forward ? "attach object" : "detach object");
		PropertyMap& p = attach->properties();
		p.declare<std::string>("eef");
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT, { "eef", "object" });
		attach->setCallback([this, forward](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
				const std::string& eef = p.get<std::string>("eef");
				moveit_msgs::AttachedCollisionObject obj;
				obj.object.operation = forward ? (int8_t) moveit_msgs::CollisionObject::ADD
				                               : (int8_t) moveit_msgs::CollisionObject::REMOVE;
				obj.link_name = scene->getRobotModel()->getEndEffector(eef)->getEndEffectorParentGroup().second;
				obj.object.id = p.get<std::string>("object");
				scene->processAttachedCollisionObjectMsg(obj);
			});
		insert(std::unique_ptr<ModifyPlanningScene>(attach), insertion_position);
	}
}

void SimpleGraspBase::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	model_ = robot_model;
	SerialContainer::init(robot_model);
}

void SimpleGraspBase::setMonitoredStage(Stage* monitored)
{
	generator_->setMonitoredStage(monitored);
}

void SimpleGraspBase::setIKFrame(const Eigen::Affine3d& pose, const std::string& link) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setIKFrame(pose_msg);
}

SimpleGrasp::SimpleGrasp(std::unique_ptr<MonitoringGenerator>&& generator, const std::string& name)
   : SimpleGraspBase(name)
{
	setup(std::move(generator), true);
}

SimpleUnGrasp::SimpleUnGrasp(std::unique_ptr<MonitoringGenerator>&& generator, const std::string& name)
   : SimpleGraspBase(name) {
	setup(std::move(generator), false);
}

} } }
