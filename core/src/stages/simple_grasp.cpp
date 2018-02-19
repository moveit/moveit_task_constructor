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

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

SimpleGrasp::SimpleGrasp(const std::string& name)
   : SerialContainer(name)
{
	PropertyMap* p = &properties();
	// propagate properties from children
	p->declare<std::string>("eef", "end effector name");
	p->declare<std::string>("object", "name of object to grasp");
	p->configureInitFrom(Stage::PARENT, { "eef", "object" });

	p->declare<std::string>("pregrasp", "pre-grasp pose of eef");
	p->declare<std::string>("grasp", "grasp pose of eef");

	p->declare<geometry_msgs::TransformStamped>("tool_to_grasp_tf", geometry_msgs::TransformStamped(),
	                                            "transform from robot tool frame to grasp frame");
	p->declare<double>("angle_delta", 0.1, "angular steps (rad)");

	p->declare<uint32_t>("max_ik_solutions", 1);
	p->declare<double>("timeout", 1.0);

	{
		auto gengrasp = std::make_unique<GenerateGraspPose>("generate grasp pose");
		p = &gengrasp->properties();
		p->configureInitFrom(Stage::PARENT, { "eef", "object", "angle_delta" });
		p->property("eef").configureInitFrom(Stage::PARENT, "eef");
		p->property("named_pose").configureInitFrom(Stage::PARENT, "pregrasp");
		p->property("object").configureInitFrom(Stage::PARENT, "object");

		auto ik = std::make_unique<ComputeIK>("compute ik", std::move(gengrasp));
		p = &ik->properties();
		p->configureInitFrom(Stage::PARENT, { "eef", "max_ik_solutions", "timeout" });
		insert(std::move(ik));
	}
	{
		auto allow_touch = std::make_unique<ModifyPlanningScene>("enable object collision");
		p = &allow_touch->properties();
		p->configureInitFrom(Stage::PARENT, { "eef", "object" });
		allow_touch->restrictDirection(ModifyPlanningScene::FORWARD);

		allow_touch->setCallback([this](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
			collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
			const std::string& eef = p.get<std::string>("eef");
			const std::string& object = p.get<std::string>("object");
			acm.setEntry(object, scene->getRobotModel()->getEndEffector(eef)
			             ->getLinkModelNamesWithCollisionGeometry(), true);
		});
		insert(std::move(allow_touch));
	}
	{
		auto pipeline = std::make_shared<solvers::PipelinePlanner>();
		pipeline->setTimeout(8.0);
		pipeline->setPlannerId("RRTConnectkConfigDefault");

		auto move = std::make_unique<MoveTo>("close gripper", pipeline);
		move->restrictDirection(MoveTo::FORWARD);
		p = &move->properties();
		p->property("joint_pose").configureInitFrom(Stage::PARENT, "grasp");
		p->property("group").configureInitFrom(Stage::PARENT, [this](const PropertyMap& parent_map){
			const std::string& eef = parent_map.get<std::string>("eef");
			const moveit::core::JointModelGroup* jmg = model_->getEndEffector(eef);
			return boost::any(jmg->getName());
		});
		insert(std::move(move));
	}
}

void SimpleGrasp::init(const planning_scene::PlanningSceneConstPtr& scene)
{
	model_ = scene->getRobotModel();
	SerialContainer::init(scene);
}

void SimpleGrasp::setToolToGraspTF(const Eigen::Affine3d& transform, const std::string& link) {
	geometry_msgs::TransformStamped stamped;
	stamped.header.frame_id = link;
	stamped.child_frame_id = "grasp_frame";
	tf::transformEigenToMsg(transform, stamped.transform);
	setToolToGraspTF(stamped);
}

} } }
