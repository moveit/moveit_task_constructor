/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner */

#include <moveit/task_constructor/stages/move.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace stages {

Move::Move(std::string name)
   : Connecting(name),
     timeout_(5.0)
{}

void Move::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Connecting::init(scene);
	planner_ = Task::createPlanner(scene->getRobotModel());
}

void Move::setGroup(std::string group){
	group_= group;
	mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_);
}

void Move::setPlannerId(std::string planner){
	planner_id_= planner;
}

void Move::setTimeout(double timeout){
	timeout_= timeout;
}

bool Move::compute(const InterfaceState &from, const InterfaceState &to) {
	mgi_->setJointValueTarget(to.scene()->getCurrentState());
	if( !planner_id_.empty() )
		mgi_->setPlannerId(planner_id_);
	mgi_->setPlanningTime(timeout_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep();
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(from.scene(), req, res))
		return false;

	// finish stage
	connect(from, to, res.trajectory_);

	return true;
}

} } }
