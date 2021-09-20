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
#include <assert.h>

namespace moveit {
namespace task_constructor {

planning_scene::PlanningSceneConstPtr ensureUpdated(const planning_scene::PlanningScenePtr& scene) {
	// ensure scene's state is updated
	if (scene->getCurrentState().dirty())
		scene->getCurrentStateNonConst().update();
	return scene;
}

InterfaceState::InterfaceState(const planning_scene::PlanningScenePtr& ps) : InterfaceState(ensureUpdated(ps)) {}

InterfaceState::InterfaceState(const planning_scene::PlanningSceneConstPtr& ps)
  : scene_(ps), priority_(Priority(0, 0.0)) {
	if (scene_->getCurrentState().dirty())
		ROS_ERROR_NAMED("InterfaceState", "Dirty PlanningScene! Please only forward clean ones into InterfaceState.");
}

InterfaceState::InterfaceState(const planning_scene::PlanningSceneConstPtr& ps, const Priority& p)
  : InterfaceState(ps) {
	priority_ = p;
}

InterfaceState::InterfaceState(const InterfaceState& other)
  : scene_(other.scene_), properties_(other.properties_), priority_(other.priority_) {}

bool InterfaceState::Priority::operator<(const InterfaceState::Priority& other) const {
	// first order by status if that differs
	if (status() != other.status())
		return status() < other.status();

	// then by depth if that differs
	if (depth() != other.depth())
		return depth() > other.depth();  // larger depth = smaller prio!

	// finally by cost
	return cost() < other.cost();
}

Interface::Interface(const Interface::NotifyFunction& notify) : notify_(notify) {}

// Announce a new InterfaceState
void Interface::add(InterfaceState& state) {
	// require valid scene
	assert(state.scene());
	// incoming and outgoing must not contain elements both
	assert(state.incomingTrajectories().empty() || state.outgoingTrajectories().empty());
	// if non-empty, incoming or outgoing should have exactly one solution element
	assert(state.incomingTrajectories().empty() || state.incomingTrajectories().size() == 1);
	assert(state.outgoingTrajectories().empty() || state.outgoingTrajectories().size() == 1);
	// state can only be added once to an interface
	assert(state.owner_ == nullptr);

	// move state to a list node
	std::list<InterfaceState*> container;
	Interface::iterator it = container.insert(container.end(), &state);
	it->owner_ = this;

	// if either incoming or outgoing is defined, derive priority from there
	if (!state.incomingTrajectories().empty())
		it->priority_ = InterfaceState::Priority(1, state.incomingTrajectories().front()->cost());
	else if (!state.outgoingTrajectories().empty())
		it->priority_ = InterfaceState::Priority(1, state.outgoingTrajectories().front()->cost());
	else {  // otherwise, assume priority was well defined before
		assert(it->priority_.enabled());
		assert(it->priority_.depth() >= 1u);
	}

	// move list node into interface's state list (sorted by priority)
	moveFrom(it, container);
	// and finally call notify callback
	if (notify_)
		notify_(it, false);
}

Interface::container_type Interface::remove(iterator it) {
	container_type result;
	moveTo(it, result, result.end());
	it->owner_ = nullptr;
	return result;
}

void Interface::updatePriority(InterfaceState* state, const InterfaceState::Priority& priority) {
	if (priority == state->priority())
		return;  // nothing to do

	auto it = std::find(begin(), end(), state);  // find iterator to state
	assert(it != end());  // state should be part of this interface
	state->priority_ = priority;  // update priority
	update(it);  // update position in ordered list
	if (notify_)
		notify_(it, true);  // notify callback
}

std::ostream& operator<<(std::ostream& os, const Interface& interface) {
	if (interface.empty())
		os << "---";
	for (const auto& istate : interface)
		os << istate->priority() << "  ";
	return os;
}
std::ostream& operator<<(std::ostream& os, const InterfaceState::Priority& prio) {
	// maps InterfaceState::Status values to output (color-changing) prefix
	static const char* prefix[] = {
		"\033[32me:",  // ENABLED - green
		"\033[33md:",  // PRUNED - yellow
		"\033[31mf:",  // FAILED - red
	};
	static const char* color_reset = "\033[m";
	os << prefix[prio.status()] << prio.depth() << ":" << prio.cost() << color_reset;
	return os;
}

void SolutionBase::setCreator(Stage* creator) {
	assert(creator_ == nullptr || creator_ == creator);  // creator must only set once
	creator_ = creator;
}

void SolutionBase::setCost(double cost) {
	cost_ = cost;
}

void SolutionBase::markAsFailure(const std::string& msg) {
	setCost(std::numeric_limits<double>::infinity());
	if (!msg.empty())
		setComment(msg + "\n" + comment());
}

void SolutionBase::fillInfo(moveit_task_constructor_msgs::SolutionInfo& info, Introspection* introspection) const {
	info.id = introspection ? introspection->solutionId(*this) : 0;
	info.cost = this->cost();
	info.comment = this->comment();
	const Introspection* ci = introspection;
	info.stage_id = ci ? ci->stageId(this->creator()) : 0;

	const auto& markers = this->markers();
	info.markers.resize(markers.size());
	std::copy(markers.begin(), markers.end(), info.markers.begin());
}

void SubTrajectory::fillMessage(moveit_task_constructor_msgs::Solution& msg, Introspection* introspection) const {
	msg.sub_trajectory.emplace_back();
	moveit_task_constructor_msgs::SubTrajectory& t = msg.sub_trajectory.back();
	SolutionBase::fillInfo(t.info, introspection);

	if (trajectory())
		trajectory()->getRobotTrajectoryMsg(t.trajectory);

	this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);
}

double SubTrajectory::computeCost(const CostTerm& f, std::string& comment) const {
	return f(*this, comment);
}

void SolutionSequence::push_back(const SolutionBase& solution) {
	subsolutions_.push_back(&solution);
}

void SolutionSequence::fillMessage(moveit_task_constructor_msgs::Solution& msg, Introspection* introspection) const {
	moveit_task_constructor_msgs::SubSolution sub_msg;
	SolutionBase::fillInfo(sub_msg.info, introspection);

	// Usually subsolutions originate from another stage than this solution.
	// However, the Connect stage will announce sub solutions created by itself
	// (only in case of merge failure). To not confuse the display's SolutionModel
	// we do not publish own IDs for them, but set those IDs to zero.
	sub_msg.sub_solution_id.reserve(subsolutions_.size());
	if (introspection) {
		for (const SolutionBase* s : subsolutions_) {
			// skip sub solutions with same creator as this
			if (s->creator() == this->creator())
				continue;
			sub_msg.sub_solution_id.push_back(introspection->solutionId(*s));
		}
		msg.sub_solution.push_back(sub_msg);
	}

	msg.sub_trajectory.reserve(msg.sub_trajectory.size() + subsolutions_.size());
	for (const SolutionBase* s : subsolutions_) {
		size_t current = msg.sub_trajectory.size();
		s->fillMessage(msg, introspection);

		// zero IDs of sub solutions with same creator as this
		if (s->creator() == this->creator()) {
			auto it = msg.sub_trajectory.begin();
			auto end = msg.sub_trajectory.end();
			std::advance(it, current);
			for (; it != end; ++it)
				it->info.id = 0;
		}
	}
}

double SolutionSequence::computeCost(const CostTerm& f, std::string& comment) const {
	return f(*this, comment);
}

void WrappedSolution::fillMessage(moveit_task_constructor_msgs::Solution& solution,
                                  Introspection* introspection) const {
	wrapped_->fillMessage(solution, introspection);

	// prepend this solutions info as a SubSolution msg
	moveit_task_constructor_msgs::SubSolution sub_msg;
	SolutionBase::fillInfo(sub_msg.info, introspection);
	sub_msg.sub_solution_id.push_back(introspection ? introspection->solutionId(*wrapped_) : 0);
	solution.sub_solution.insert(solution.sub_solution.begin(), std::move(sub_msg));
}

double WrappedSolution::computeCost(const CostTerm& f, std::string& comment) const {
	return f(*this, comment);
}

}  // namespace task_constructor
}  // namespace moveit
