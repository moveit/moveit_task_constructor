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
   : Connecting(name)
{
	auto& p = properties();
	p.declare<double>("timeout", 5.0, "planning timeout");
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("planner", "", "planner id");
}

void Move::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	Connecting::init(robot_model);
	planner_ = Task::createPlanner(robot_model);
}

void Move::setGroup(const std::string& group){
	setProperty("group", group);
}

void Move::setPlannerId(const std::string& planner){
	setProperty("planner", planner);
}

void Move::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

bool Move::compute(const InterfaceState &from, const InterfaceState &to) {
	const auto& props = properties();
	moveit::planning_interface::MoveGroupInterface mgi(props.get<std::string>("group"));
	mgi.setJointValueTarget(to.scene()->getCurrentState());

	const std::string planner_id = props.get<std::string>("planner");
	if( !planner_id.empty() )
		mgi.setPlannerId(planner_id);
	mgi.setPlanningTime(props.get<double>("timeout"));

	::planning_interface::MotionPlanRequest req;
	mgi.constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep(); // TODO: get rid of this!
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(from.scene(), req, res))
		return false;

	// finish stage
	connect(from, to, SubTrajectory(res.trajectory_));

	return true;
}

} } }
