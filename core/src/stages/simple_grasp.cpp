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
	{
		auto gengrasp = std::make_unique<GenerateGraspPose>("generate grasp pose");
		grasp_generator_ = gengrasp.get();

		auto ik = std::make_unique<ComputeIK>("compute ik", std::move(gengrasp));
		const std::initializer_list<std::string>& grasp_prop_names = { "eef", "pregrasp", "object", "angle_delta", "tool_to_grasp_tf" };
		ik->exposePropertiesOfChild(0, grasp_prop_names);
		insert(std::move(ik));

		exposePropertiesOfChild(-1, grasp_prop_names);
		exposePropertiesOfChild(-1, { "max_ik_solutions", "timeout" });
	}
	{
		auto allow_touch = std::make_unique<ModifyPlanningScene>("enable object collision");
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
		insert(std::move(allow_touch));
	}
	{
		auto pipeline = std::make_shared<solvers::PipelinePlanner>();
		pipeline->setTimeout(8.0);
		pipeline->setPlannerId("RRTConnectkConfigDefault");

		auto move = std::make_unique<MoveTo>("close gripper", pipeline);
		move->restrictDirection(MoveTo::FORWARD);
		PropertyMap& p = move->properties();
		p.property("group").configureInitFrom(Stage::PARENT, [this](const PropertyMap& parent_map){
			const std::string& eef = parent_map.get<std::string>("eef");
			const moveit::core::JointModelGroup* jmg = model_->getEndEffector(eef);
			return boost::any(jmg->getName());
		});
		insert(std::move(move));
		exposePropertyOfChildAs(-1, "joint_pose", "grasp");
	}
}

void SimpleGrasp::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	model_ = robot_model;
	SerialContainer::init(robot_model);
}

void SimpleGrasp::setMonitoredStage(Stage* monitored)
{
	grasp_generator_->setMonitoredStage(monitored);
}

void SimpleGrasp::setToolToGraspTF(const Eigen::Affine3d& transform, const std::string& link) {
	geometry_msgs::TransformStamped stamped;
	stamped.header.frame_id = link;
	stamped.child_frame_id = "grasp_frame";
	tf::transformEigenToMsg(transform, stamped.transform);
	setToolToGraspTF(stamped);
}

} } }
