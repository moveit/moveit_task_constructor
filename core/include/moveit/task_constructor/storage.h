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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Classes to store and pass partial solutions between stages
*/

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/task_constructor/properties.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <vector>
#include <deque>
#include <cassert>
#include <functional>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit { namespace task_constructor {

class SolutionBase;
MOVEIT_CLASS_FORWARD(InterfaceState)
MOVEIT_CLASS_FORWARD(Interface)
typedef std::weak_ptr<Interface> InterfaceWeakPtr;
MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(Introspection)


/** InterfaceState describes a potential start or goal state for a planning stage.
 *
 *  A start or goal state for planning is essentially defined by the state of a planning scene.
 */
class InterfaceState {
	friend class SolutionBase; // addIncoming() / addOutgoing() should be called only by SolutionBase
public:
	// TODO turn this into priority queue
	typedef std::deque<SolutionBase*> Solutions;

	/// create an InterfaceState from a planning scene
	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps);

	/// copy an existing InterfaceState, but not including incoming/outgoing trajectories
	InterfaceState(const InterfaceState& existing);

	inline const planning_scene::PlanningSceneConstPtr& scene() const { return scene_; }
	inline const Solutions& incomingTrajectories() const { return incoming_trajectories_; }
	inline const Solutions& outgoingTrajectories() const { return outgoing_trajectories_; }

	PropertyMap& properties() { return properties_; }
	const PropertyMap& properties() const { return properties_; }

private:
	// these methods should be only called by SolutionBase::set[Start|End]State()
	inline void addIncoming(SolutionBase* t) { incoming_trajectories_.push_back(t); }
	inline void addOutgoing(SolutionBase* t) { outgoing_trajectories_.push_back(t); }

private:
	planning_scene::PlanningSceneConstPtr scene_;
	PropertyMap properties_;
	Solutions incoming_trajectories_;
	Solutions outgoing_trajectories_;
};


/** Interface provides a list of InterfaceStates available as input for a stage.
 *
 *  This is essentially an adaptor to a container class, to allow for notification
 *  of the interface's owner when new states become available
 */
class Interface : protected std::list<InterfaceState> {
public:
	typedef std::list<InterfaceState> container_type;
	typedef std::function<void(const container_type::iterator&)> NotifyFunction;
	Interface(const NotifyFunction &notify = NotifyFunction());

	// add a new InterfaceState, connect the trajectory (either incoming or outgoing) to the newly created state
	// and finally run the notify callback
	container_type::iterator add(InterfaceState &&state, SolutionBase* incoming, SolutionBase* outgoing);

	// clone an existing InterfaceState, but without its incoming/outgoing connections
	container_type::iterator clone(const InterfaceState &state);

	using container_type::value_type;
	using container_type::reference;
	using container_type::const_reference;

	using container_type::iterator;
	using container_type::const_iterator;
	using container_type::reverse_iterator;
	using container_type::const_reverse_iterator;

	using container_type::empty;
	using container_type::size;
	using container_type::clear;
	using container_type::front;
	using container_type::back;

	using container_type::begin;
	using container_type::cbegin;
	using container_type::end;
	using container_type::cend;
	using container_type::rbegin;
	using container_type::crbegin;
	using container_type::rend;
	using container_type::crend;

private:
	const NotifyFunction notify_;
};


class StagePrivate;
class SubTrajectory;

/// SolutionTrajectory is composed of a series of SubTrajectories
typedef std::vector<const SubTrajectory*> SolutionTrajectory;

class SolutionBase {
public:
	inline const InterfaceState* start() const { return start_; }
	inline const InterfaceState* end() const { return end_; }

	inline void setStartState(const InterfaceState& state){
		assert(start_ == NULL);
		start_ = &state;
		const_cast<InterfaceState&>(state).addOutgoing(this);
	}

	inline void setEndState(const InterfaceState& state){
		assert(end_ == NULL);
		end_ = &state;
		const_cast<InterfaceState&>(state).addIncoming(this);
	}

	inline const StagePrivate* creator() const { return creator_; }
	void setCreator(StagePrivate* creator);

	inline double cost() const { return cost_; }
	void setCost(double cost);
	inline bool isFailure() const { return !std::isfinite(cost_); }

	const std::string& name() const { return name_; }
	void setName(const std::string& name) { name_ = name; }

	auto& markers() { return markers_; }
	const auto& markers() const { return markers_; }

	/// append this solution to Solution msg
	virtual void fillMessage(moveit_task_constructor_msgs::Solution &solution,
	                         Introspection* introspection = nullptr) const = 0;

	/// order solutions by their cost
	bool operator<(const SolutionBase& other) const {
		return this->cost_ < other.cost_;
	}

protected:
	SolutionBase(StagePrivate* creator = nullptr, double cost = 0.0)
	   : creator_(creator), cost_(cost)
	{
	}

private:
	// back-pointer to creating stage, allows to access sub-solutions
	StagePrivate *creator_;
	// associated cost
	double cost_;
	// trajectories could have a name, e.g. a generator could name its solutions
	std::string name_;
	// additional markers
	std::deque<visualization_msgs::Marker> markers_;

	// begin and end InterfaceState of this solution/trajectory
	const InterfaceState* start_ = nullptr;
	const InterfaceState* end_ = nullptr;
};


// SubTrajectory connects interface states of ComputeStages
class SubTrajectory : public SolutionBase {
public:
	SubTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory = robot_trajectory::RobotTrajectoryPtr())
	   : SolutionBase(), trajectory_(trajectory)
	{}

	robot_trajectory::RobotTrajectoryConstPtr trajectory() const { return trajectory_; }

	void fillMessage(moveit_task_constructor_msgs::Solution &msg,
	                 Introspection* introspection = nullptr) const override;

private:
	// actual trajectory, might be empty
	const robot_trajectory::RobotTrajectoryPtr trajectory_;
};


enum TraverseDirection { FORWARD, BACKWARD };
template <TraverseDirection dir>
const InterfaceState::Solutions& trajectories(const SolutionBase &start);

template <> inline
const InterfaceState::Solutions& trajectories<FORWARD>(const SolutionBase &solution) {
	return solution.end()->outgoingTrajectories();
}
template <> inline
const InterfaceState::Solutions& trajectories<BACKWARD>(const SolutionBase &solution) {
	return solution.start()->incomingTrajectories();
}

} }
