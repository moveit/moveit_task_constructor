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
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/task_constructor/utils.h>

#include <list>
#include <vector>
#include <deque>
#include <cassert>
#include <functional>
#include <cmath>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory);
}

namespace moveit {
namespace task_constructor {

class SolutionBase;
MOVEIT_CLASS_FORWARD(InterfaceState);
MOVEIT_CLASS_FORWARD(Interface);
MOVEIT_CLASS_FORWARD(Stage);
MOVEIT_CLASS_FORWARD(Introspection);

/** InterfaceState describes a potential start or goal state for a planning stage.
 *
 *  A start or goal state for planning is essentially defined by the state of a planning scene.
 */
class InterfaceState
{
	friend class SolutionBase;  // addIncoming() / addOutgoing() should be called only by SolutionBase
	friend class Interface;  // allow Interface to set owner_ and priority_
	friend class ContainerBasePrivate;  // allow setting priority_ for pruning

public:
	enum Status
	{
		ENABLED,  // state is actively considered during planning
		ARMED,  // disabled state in a Connecting interface that will become re-enabled with a new opposite state
		PRUNED,  // disabled state on a pruned solution branch
	};
	static const char* colorForStatus(unsigned int s) { return STATUS_COLOR_[s]; }

	/** InterfaceStates are ordered according to two values:
	 *  Depth of interlinked trajectory parts and accumulated trajectory costs along that path.
	 *  Preference ordering considers high-depth first and within same depth, minimal cost paths.
	 */
	struct Priority : std::tuple<Status, unsigned int, double>
	{
		Priority(unsigned int depth, double cost, Status status)
		  : std::tuple<Status, unsigned int, double>(status, depth, cost) {}
		Priority(unsigned int depth, double cost) : Priority(depth, cost, std::isfinite(cost) ? ENABLED : PRUNED) {}
		// Constructor copying depth and cost, but modifying its status
		Priority(const Priority& other, Status status) : Priority(other.depth(), other.cost(), status) {}

		inline Status status() const { return std::get<0>(*this); }
		inline bool enabled() const { return std::get<0>(*this) == ENABLED; }

		inline unsigned int depth() const { return std::get<1>(*this); }
		inline double cost() const { return std::get<2>(*this); }

		// add priorities
		Priority operator+(const Priority& other) const {
			return Priority(depth() + other.depth(), cost() + other.cost(), std::min(status(), other.status()));
		}
		// comparison operators
		bool operator<(const Priority& rhs) const;
		inline bool operator>(const Priority& rhs) const { return rhs < *this; }
		inline bool operator<=(const Priority& rhs) const { return !(rhs < *this); }
		inline bool operator>=(const Priority& rhs) const { return !(*this < rhs); }
	};
	using Solutions = std::deque<SolutionBase*>;

	/// create an InterfaceState from a planning scene
	InterfaceState(const planning_scene::PlanningScenePtr& ps);
	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps);

	/// provide an initial priority for the state (for internal use only)
	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps, const Priority& p);

	/// copy an existing InterfaceState, but not including incoming/outgoing trajectories
	InterfaceState(const InterfaceState& other);
	InterfaceState(InterfaceState&& other) = default;
	InterfaceState& operator=(const InterfaceState& other) = default;

	inline const planning_scene::PlanningSceneConstPtr& scene() const { return scene_; }
	inline const Solutions& incomingTrajectories() const { return incoming_trajectories_; }
	inline const Solutions& outgoingTrajectories() const { return outgoing_trajectories_; }

	PropertyMap& properties() { return properties_; }
	const PropertyMap& properties() const { return properties_; }

	/// states are ordered by priority
	inline bool operator<(const InterfaceState& other) const { return this->priority_ < other.priority_; }

	inline const Priority& priority() const { return priority_; }
	/// Update priority and call owner's notify() if possible
	void updatePriority(const InterfaceState::Priority& priority);
	/// Update status, but keep current priority
	void updateStatus(Status status);

	Interface* owner() const { return owner_; }

private:
	// these methods should be only called by SolutionBase::set[Start|End]State()
	inline void addIncoming(SolutionBase* t) { incoming_trajectories_.push_back(t); }
	inline void addOutgoing(SolutionBase* t) { outgoing_trajectories_.push_back(t); }
	// Set new priority without updating the owning interface (USE WITH CARE)
	inline void setPriority(const Priority& prio) { priority_ = prio; }

private:
	static const char* STATUS_COLOR_[];
	planning_scene::PlanningSceneConstPtr scene_;
	PropertyMap properties_;
	/// trajectories which are *timewise before* this state
	Solutions incoming_trajectories_;
	/// trajectories which are *timewise after* this state
	Solutions outgoing_trajectories_;

	// members needed for priority scheduling in Interface list
	Priority priority_;
	Interface* owner_ = nullptr;  // allow update of priority
};

/** Interface provides a cost-sorted list of InterfaceStates available as input for a stage. */
class Interface : public ordered<InterfaceState*>
{
	using base_type = ordered<InterfaceState*>;

public:
	// iterators providing convinient access to stored InterfaceState
	class iterator : public base_type::iterator
	{
	public:
		iterator() = default;
		iterator(base_type::iterator other) : base_type::iterator(other) {}

		InterfaceState& operator*() const noexcept { return *base_type::iterator::operator*(); }

		InterfaceState* operator->() const noexcept { return base_type::iterator::operator*(); }
	};
	class const_iterator : public base_type::const_iterator
	{
	public:
		const_iterator(base_type::const_iterator other) : base_type::const_iterator(other) {}
		const_iterator(base_type::iterator other) : base_type::const_iterator(other) {}

		const InterfaceState& operator*() const noexcept { return *base_type::const_iterator::operator*(); }

		const InterfaceState* operator->() const noexcept { return base_type::const_iterator::operator*(); }
	};

	enum Direction
	{
		FORWARD,
		BACKWARD,
	};
	enum Update
	{
		STATUS = 1 << 0,
		PRIORITY = 1 << 1,
		ALL = STATUS | PRIORITY,
	};
	using UpdateFlags = utils::Flags<Update>;
	using NotifyFunction = std::function<void(iterator, UpdateFlags)>;

	class DisableNotify
	{
		Interface& if_;
		Interface::NotifyFunction old_;

	public:
		DisableNotify(Interface& i) : if_(i) { old_.swap(if_.notify_); }
		~DisableNotify() { old_.swap(if_.notify_); }
	};
	friend class DisableNotify;

	Interface(const NotifyFunction& notify = NotifyFunction());

	/// add a new InterfaceState
	void add(InterfaceState& state);

	/// remove a state from the interface and return it as a one-element list
	container_type remove(iterator it);

	/// update state's priority (and call notify_ if it really has changed)
	void updatePriority(InterfaceState* state, const InterfaceState::Priority& priority);
	inline bool notifyEnabled() const { return static_cast<bool>(notify_); }

private:
	NotifyFunction notify_;

	// restrict access to some functions to ensure consistency
	// (we need to set/unset InterfaceState::owner_)
	using base_type::erase;
	using base_type::insert;
	using base_type::moveFrom;
	using base_type::moveTo;
	using base_type::remove_if;
};

std::ostream& operator<<(std::ostream& os, const InterfaceState::Priority& prio);
std::ostream& operator<<(std::ostream& os, const Interface& interface);
std::ostream& operator<<(std::ostream& os, Interface::Direction dir);

/// Find index of the iterator in the container. Counting starts at 1. Zero corresponds to not found.
template <typename T>
size_t getIndex(const T& container, typename T::const_iterator search) {
	size_t index = 1;
	for (typename T::const_iterator it = container.begin(), end = container.end(); it != end; ++it, ++index)
		if (it == search)
			return index;
	return 0;
}

class CostTerm;
class StagePrivate;
class ContainerBasePrivate;
struct TmpSolutionContext;
/// abstract base class for solutions (primitive and sequences)
class SolutionBase
{
	friend ContainerBasePrivate;
	friend TmpSolutionContext;

public:
	virtual ~SolutionBase() = default;

	inline const InterfaceState* start() const { return start_; }
	inline const InterfaceState* end() const { return end_; }

	/** Set the solution's start_state_
	 *
	 * Must be called only once, because it registers the solution with the state.
	 */
	inline void setStartState(const InterfaceState& state) {
		assert(start_ == nullptr);
		start_ = &state;
		const_cast<InterfaceState&>(state).addOutgoing(this);
	}

	/** Set the solution's end_state_
	 *
	 * Must be called only once, because it registers the solution with the state.
	 */
	inline void setEndState(const InterfaceState& state) {
		assert(end_ == nullptr);
		end_ = &state;
		const_cast<InterfaceState&>(state).addIncoming(this);
	}

	inline const Stage* creator() const { return creator_; }
	void setCreator(Stage* creator);

	inline double cost() const { return cost_; }
	void setCost(double cost);
	void markAsFailure(const std::string& msg = std::string());
	inline bool isFailure() const { return !std::isfinite(cost_); }

	const std::string& comment() const { return comment_; }
	void setComment(const std::string& comment) { comment_ = comment; }

	const std::string& plannerId() const { return planner_id_; }
	void setPlannerId(const std::string& planner_id) { planner_id_ = planner_id; }

	auto& markers() { return markers_; }
	const auto& markers() const { return markers_; }

	/// convert solution to message
	void toMsg(moveit_task_constructor_msgs::msg::Solution& solution, Introspection* introspection = nullptr) const;
	/// append this solution to Solution msg
	virtual void appendTo(moveit_task_constructor_msgs::msg::Solution& solution,
	                      Introspection* introspection = nullptr) const = 0;
	void fillInfo(moveit_task_constructor_msgs::msg::SolutionInfo& info, Introspection* introspection = nullptr) const;

	/// required to dispatch to type-specific CostTerm methods via vtable
	virtual double computeCost(const CostTerm& cost, std::string& comment) const = 0;

	/// order solutions by their cost
	bool operator<(const SolutionBase& other) const { return this->cost_ < other.cost_; }

protected:
	SolutionBase(Stage* creator = nullptr, double cost = 0.0, std::string comment = "", std::string planner_id = "")
	  : creator_(creator), cost_(cost), comment_(std::move(comment)), planner_id_(std::move(planner_id)) {}

private:
	// back-pointer to creating stage, allows to access sub-solutions
	Stage* creator_;
	// associated cost
	double cost_;
	// comment for this solution, e.g. explanation of failure
	std::string comment_;
	// name of the planner used to create this solution
	std::string planner_id_;
	// markers for this solution, e.g. target frame or collision indicators
	std::deque<visualization_msgs::msg::Marker> markers_;

	// begin and end InterfaceState of this solution/trajectory
	const InterfaceState* start_ = nullptr;
	const InterfaceState* end_ = nullptr;
};
MOVEIT_CLASS_FORWARD(SolutionBase);

/// SubTrajectory connects interface states of ComputeStages
class SubTrajectory : public SolutionBase
{
public:
	SubTrajectory(
	    const robot_trajectory::RobotTrajectoryConstPtr& trajectory = robot_trajectory::RobotTrajectoryConstPtr(),
	    double cost = 0.0, std::string comment = "", std::string planner_id = "")
	  : SolutionBase(nullptr, cost, std::move(comment), std::move(planner_id)), trajectory_(trajectory) {}

	robot_trajectory::RobotTrajectoryConstPtr trajectory() const { return trajectory_; }
	void setTrajectory(const robot_trajectory::RobotTrajectoryPtr& t) { trajectory_ = t; }

	void appendTo(moveit_task_constructor_msgs::msg::Solution& msg,
	              Introspection* introspection = nullptr) const override;

	double computeCost(const CostTerm& cost, std::string& comment) const override;

	static SubTrajectory failure(const std::string& msg) {
		SubTrajectory s;
		s.markAsFailure(msg);
		return s;
	}

private:
	// actual trajectory, might be empty
	robot_trajectory::RobotTrajectoryConstPtr trajectory_;
};
MOVEIT_CLASS_FORWARD(SubTrajectory);

/** Sequence of individual sub solutions
 *
 * A solution sequence describes a solution that is composed from several individual
 * sub solutions that need to be chained together to yield the overall solutions.
 */
class SolutionSequence : public SolutionBase
{
public:
	using container_type = std::vector<const SolutionBase*>;

	explicit SolutionSequence() : SolutionBase() {}
	SolutionSequence(container_type&& subsolutions, double cost = 0.0, Stage* creator = nullptr)
	  : SolutionBase(creator, cost), subsolutions_(std::move(subsolutions)) {}

	void push_back(const SolutionBase& solution);

	/// append all subsolutions to solution
	void appendTo(moveit_task_constructor_msgs::msg::Solution& msg, Introspection* introspection) const override;

	double computeCost(const CostTerm& cost, std::string& comment) const override;

	const container_type& solutions() const { return subsolutions_; }

	inline const InterfaceState* internalStart() const { return subsolutions_.front()->start(); }
	inline const InterfaceState* internalEnd() const { return subsolutions_.back()->end(); }

private:
	/// series of sub solutions
	container_type subsolutions_;
};
MOVEIT_CLASS_FORWARD(SolutionSequence);

/** Wrap an existing solution
 *
 * used by parallel containers and wrappers.
 *
 * This essentially wraps a solution of a child and thus allows
 * for new clones of start / end states, which in turn will
 * have separate incoming/outgoing trajectories */
class WrappedSolution : public SolutionBase
{
public:
	explicit WrappedSolution(Stage* creator, const SolutionBase* wrapped, double cost, std::string comment)
	  : SolutionBase(creator, cost, std::move(comment)), wrapped_(wrapped) {}
	explicit WrappedSolution(Stage* creator, const SolutionBase* wrapped, double cost)
	  : SolutionBase(creator, cost), wrapped_(wrapped) {}
	explicit WrappedSolution(Stage* creator, const SolutionBase* wrapped)
	  : WrappedSolution(creator, wrapped, wrapped->cost()) {}
	void appendTo(moveit_task_constructor_msgs::msg::Solution& solution,
	              Introspection* introspection = nullptr) const override;

	double computeCost(const CostTerm& cost, std::string& comment) const override;

	const SolutionBase* wrapped() const { return wrapped_; }

private:
	const SolutionBase* wrapped_;
};

/// Trait to retrieve the end (FORWARD) or start (BACKWARD) state of a given solution
template <Interface::Direction dir>
const InterfaceState* state(const SolutionBase& solution);
template <>
inline const InterfaceState* state<Interface::FORWARD>(const SolutionBase& solution) {
	return solution.end();
}
template <>
inline const InterfaceState* state<Interface::BACKWARD>(const SolutionBase& solution) {
	return solution.start();
}

/// Trait to retrieve outgoing (FORWARD) or incoming (BACKWARD) solution segments of a given state
template <Interface::Direction dir>
const InterfaceState::Solutions& trajectories(const InterfaceState& state);
template <>
inline const InterfaceState::Solutions& trajectories<Interface::FORWARD>(const InterfaceState& state) {
	return state.outgoingTrajectories();
}
template <>
inline const InterfaceState::Solutions& trajectories<Interface::BACKWARD>(const InterfaceState& state) {
	return state.incomingTrajectories();
}

/// Trait to retrieve opposite direction (FORWARD <-> BACKWARD)
template <Interface::Direction dir>
constexpr Interface::Direction opposite();
template <>
inline constexpr Interface::Direction opposite<Interface::FORWARD>() {
	return Interface::BACKWARD;
}
template <>
inline constexpr Interface::Direction opposite<Interface::BACKWARD>() {
	return Interface::FORWARD;
}
}  // namespace task_constructor
}  // namespace moveit

namespace std {
// comparison for pointers to SolutionBase: compare based on value
template <>
struct less<moveit::task_constructor::SolutionBase*>
{
	bool operator()(const moveit::task_constructor::SolutionBase* x, const moveit::task_constructor::SolutionBase* y) {
		return *x < *y;
	}
};
}  // namespace std
