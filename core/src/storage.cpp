/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner, Robert Haschke */

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

InterfaceState::InterfaceState(const planning_scene::PlanningSceneConstPtr &ps)
   : scene_(ps)
{
}

InterfaceState::InterfaceState(const InterfaceState &existing)
   : scene_(existing.scene()), properties_(existing.properties())
{
}

Interface::Interface(const Interface::NotifyFunction &notify)
   : notify_(notify)
{}

Interface::iterator Interface::add(InterfaceState &&state, SolutionBase* incoming, SolutionBase* outgoing) {
	if (!state.incomingTrajectories().empty() || !state.outgoingTrajectories().empty())
		throw std::runtime_error("expecting empty incoming/outgoing trajectories");
	if (!state.scene())
		throw std::runtime_error("expecting valid planning scene");

	assert(bool(incoming) ^ bool(outgoing)); // either incoming or outgoing is set
	emplace_back(state);
	iterator back = --end();
	// adjust subtrajectories ...
	if (incoming) incoming->setEndState(*back);
	if (outgoing) outgoing->setStartState(*back);
	// ... before calling notify callback
	if (notify_) notify_(back);
	return back;
}

Interface::iterator Interface::clone(const InterfaceState &state)
{
	emplace_back(InterfaceState(state));
	iterator back = --end();
	if (notify_) notify_(back);
	return back;
}


void SolutionBase::setCost(double cost) {
	cost_ = cost;
}


void SubTrajectory::fillMessage(moveit_task_constructor_msgs::Solution &msg,
                                Introspection *introspection) const {
	msg.sub_trajectory.emplace_back();
	moveit_task_constructor_msgs::SubTrajectory& t = msg.sub_trajectory.back();
	t.id = introspection ? introspection->solutionId(*this) : 0;
	t.cost = this->cost();
	t.name = this->name();

	const Introspection *ci = introspection;
	t.stage_id = ci ? ci->stageId(this->creator()->me()) : 0;

	if (trajectory())
		trajectory()->getRobotTrajectoryMsg(t.trajectory);

	const auto& markers = this->markers();
	t.markers.clear();
	std::copy(markers.begin(), markers.end(), std::back_inserter(t.markers));

	this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);
}


} }
