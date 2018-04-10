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
#include <moveit/task_constructor/cost_queue.h>
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
	friend class Interface; // allow Interface to set owner_ and priority_
public:
	/** InterfaceStates are ordered according to two values:
	 *  Depth of interlinked trajectory parts and accumulated trajectory costs along that path.
	 *  Preference ordering considers high-depth first and within same depth, minimal cost paths.
	 */
	struct Priority : public std::pair<unsigned int, double> {
		Priority() : Priority(0, 0.0) {}
		Priority(unsigned int depth, double cost)
		   : std::pair<unsigned int, double>(depth, cost) {}

		inline unsigned int depth() const { return this->first; }
		inline double cost() const { return this->second; }

		Priority operator+(const Priority& other) const {
			return Priority(this->depth() + other.depth(),
			                this->cost() + other.cost());
		}
		inline bool operator<(const Priority& other) const {
			/* infinite cost should always be last */
			if (std::isinf(this->cost()) && std::isinf(other.cost()))
				return this->depth() > other.depth();
			else if (std::isinf(this->cost()))
				return false;
			else if (std::isinf(other.cost()))
				return true;

			if (this->depth() == other.depth())
				return this->cost() < other.cost();
			else
				return this->depth() > other.depth();
		}
	};
	typedef std::deque<SolutionBase*> Solutions;

	/// create an InterfaceState from a planning scene
	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps);

	/// copy an existing InterfaceState, but not including incoming/outgoing trajectories
	InterfaceState(const InterfaceState& other);

	inline const planning_scene::PlanningSceneConstPtr& scene() const { return scene_; }
	inline const Solutions& incomingTrajectories() const { return incoming_trajectories_; }
	inline const Solutions& outgoingTrajectories() const { return outgoing_trajectories_; }

	PropertyMap& properties() { return properties_; }
	const PropertyMap& properties() const { return properties_; }

	/// states are ordered by priority
	inline bool operator<(const InterfaceState& other) const {
		return this->priority_ < other.priority_;
	}
	inline const Priority& priority() const { return priority_; }
	Interface* owner() const { return owner_; }

private:
	// these methods should be only called by SolutionBase::set[Start|End]State()
	inline void addIncoming(SolutionBase* t) { incoming_trajectories_.push_back(t); }
	inline void addOutgoing(SolutionBase* t) { outgoing_trajectories_.push_back(t); }

private:
	planning_scene::PlanningSceneConstPtr scene_;
	PropertyMap properties_;
	Solutions incoming_trajectories_;
	Solutions outgoing_trajectories_;

	// members needed for priority scheduling in Interface list
	Priority priority_;
	Interface* owner_ = nullptr;  // allow update of priority
};


/** Interface provides a cost-sorted list of InterfaceStates available as input for a stage. */
class Interface : public ordered<InterfaceState> {
public:
	enum Direction { FORWARD, BACKWARD, START=FORWARD, END=BACKWARD };
	typedef std::function<void(iterator it, bool updated)> NotifyFunction;
	Interface(const NotifyFunction &notify = NotifyFunction());

	/// add a new InterfaceState, connect the trajectory (either incoming or outgoing) to the newly created state
	iterator add(InterfaceState &&state, SolutionBase* incoming, SolutionBase* outgoing);

	/// clone an existing InterfaceState, but without its incoming/outgoing connections
	iterator clone(const InterfaceState &state);

	/// remove a state from the interface and return it as a one-element list
	container_type remove(iterator it);

	/// update state's priority if new priority is smaller or became infeasible and call notify_
	void updatePriority(InterfaceState *state, const InterfaceState::Priority &priority);

private:
	const NotifyFunction notify_;

	// restrict access to some functions to ensure consistency
	// (we need to set/unset InterfaceState::owner_)
	using ordered<InterfaceState>::moveTo;
	using ordered<InterfaceState>::moveFrom;
	using ordered<InterfaceState>::insert;
	using ordered<InterfaceState>::erase;
	using ordered<InterfaceState>::remove_if;
};


class StagePrivate;


/// abstract base class for solutions (primitive and sequences)
class SolutionBase {
public:
	inline const InterfaceState* start() const { return start_; }
	inline const InterfaceState* end() const { return end_; }

	template <Interface::Direction dir>
	inline const InterfaceState::Solutions& trajectories() const;

	inline void setStartState(const InterfaceState& state){
		assert(start_ == NULL);  // only allow setting once (by Stage)
		start_ = &state;
		const_cast<InterfaceState&>(state).addOutgoing(this);
	}

	inline void setEndState(const InterfaceState& state){
		assert(end_ == NULL);  // only allow setting once (by Stage)
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


/// SubTrajectory connects interface states of ComputeStages
class SubTrajectory : public SolutionBase {
public:
	SubTrajectory(const robot_trajectory::RobotTrajectoryConstPtr& trajectory = robot_trajectory::RobotTrajectoryConstPtr(),
	              double cost = 0.0)
	   : SolutionBase(nullptr, cost), trajectory_(trajectory)
	{}

	robot_trajectory::RobotTrajectoryConstPtr trajectory() const { return trajectory_; }
	void setTrajectory(const robot_trajectory::RobotTrajectoryPtr& t) { trajectory_ = t; }

	void fillMessage(moveit_task_constructor_msgs::Solution &msg,
	                 Introspection* introspection = nullptr) const override;

private:
	// actual trajectory, might be empty
	robot_trajectory::RobotTrajectoryConstPtr trajectory_;
};


/** Sequence of individual sub solutions
 *
 * A solution sequence describes a solution that is composed from several individual
 * sub solutions that need to be chained together to yield the overall solutions.
 */
class SolutionSequence : public SolutionBase {
public:
	typedef std::vector<const SolutionBase*> container_type;

	explicit SolutionSequence()
	   : SolutionBase()
	{}
	SolutionSequence(container_type&& subsolutions, double cost = 0.0, StagePrivate* creator = nullptr)
	   : SolutionBase(creator, cost), subsolutions_(subsolutions)
	{}

	void push_back(const SolutionBase& solution);

	/// append all subsolutions to solution
	void fillMessage(moveit_task_constructor_msgs::Solution &msg, Introspection *introspection) const override;

	inline const InterfaceState* internalStart() const { return subsolutions_.front()->start(); }
	inline const InterfaceState* internalEnd() const { return subsolutions_.back()->end(); }

private:
	/// series of sub solutions
	container_type subsolutions_;
};

template <> inline
const InterfaceState::Solutions& SolutionBase::trajectories<Interface::FORWARD>() const {
	return end_->outgoingTrajectories();
}
template <> inline
const InterfaceState::Solutions& SolutionBase::trajectories<Interface::BACKWARD>() const {
	return start_->incomingTrajectories();
}

} }

namespace std {
// comparison for pointers to SolutionBase: compare based on value
template<> struct less<moveit::task_constructor::SolutionBase*> {
	bool operator()(const moveit::task_constructor::SolutionBase* x,
	                const moveit::task_constructor::SolutionBase* y) {
		return *x < *y;
	}
};
}
