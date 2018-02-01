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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Connect arbitrary states by motion planning
*/

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

Connect::Connect(std::string name, const solvers::PlannerInterfacePtr& planner)
   : Connecting(name)
   , planner_(planner)
{
	auto& p = properties();
	p.declare<double>("timeout", 10.0, "planning timeout");
	p.declare<std::string>("group", "name of planning group");
}

void Connect::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Connecting::init(scene);
	planner_->init(scene->getRobotModel());
}

bool Connect::compute(const InterfaceState &from, const InterfaceState &to) {
	const auto& props = properties();
	const std::string& group = props.get<std::string>("group");
	double timeout = props.get<double>("timeout");
	const moveit::core::JointModelGroup* jmg = from.scene()->getRobotModel()->getJointModelGroup(group);

	robot_trajectory::RobotTrajectoryPtr trajectory;
	if (!planner_->plan(from.scene(), to.scene(), jmg, timeout, trajectory))
		return false;

	connect(from, to, trajectory);
	return true;
}

} } }
