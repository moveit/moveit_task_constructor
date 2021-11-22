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

/* Author: Robert Haschke
   Desc:   Private Implementation of the Containers
*/

#pragma once

#include <moveit/task_constructor/container.h>
#include <moveit/macros/class_forward.h>
#include "stage_p.h"

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/unordered_multiset_of.hpp>

#include <map>
#include <climits>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(JointModelGroup);
MOVEIT_CLASS_FORWARD(RobotState);
}  // namespace core
}  // namespace moveit

namespace moveit {
namespace task_constructor {

/* A container needs to decouple its own (external) interfaces
 * from those (internal) of its children.
 * Both, the container and the children have their own pull interfaces: starts_ and ends_.
 * The container forwards states received in its pull interfaces to the
 * corresponding interfaces of the children.
 * States pushed by children are temporarily stored in pending_backward_ and pending_forward_.
 *
 * Solutions found by the children need to be lifted from the internal level to the
 * external level. To this end, we remember the mapping from internal to external states.
 *
 * Note, that there might be many solutions connecting a single start-end pair.
 * These solutions might origin from different children (ParallelContainer)
 * or from different solution paths in a SerialContainer.
 */
class ContainerBasePrivate : public StagePrivate
{
	friend class ContainerBase;
	friend void swap(StagePrivate*& lhs, StagePrivate*& rhs);

public:
	using container_type = StagePrivate::container_type;
	using iterator = container_type::iterator;
	using const_iterator = container_type::const_iterator;
	using NonConstStageCallback = std::function<bool(Stage&, int)>;

	inline const container_type& children() const { return children_; }

	/** Retrieve iterator into children_ pointing to indexed element.
	 * Negative index counts from end, i.e. -1 is last, -2 is second last, etc. (if !for_insert)
	 * If for_insert == true, negative indexes are shifted by one: -1 is end(), -2 is --end(), etc.
	 * Contrary to std::advance(), iterator limits are considered. */
	const_iterator childByIndex(int index, bool for_insert = false) const;

	/// remove child at given iterator position, returns fals if pos is invalid
	Stage::pointer remove(ContainerBasePrivate::const_iterator pos);

	/// traversing all stages up to max_depth
	bool traverseStages(const ContainerBase::StageCallback& processor, unsigned int cur_depth,
	                    unsigned int max_depth) const;

	/// non-const version
	bool traverseStages(const NonConstStageCallback& processor, unsigned int cur_depth, unsigned int max_depth) {
		const auto& const_processor = [&processor](const Stage& stage, unsigned int depth) {
			return processor(const_cast<Stage&>(stage), depth);
		};
		return const_cast<const ContainerBasePrivate*>(this)->traverseStages(const_processor, cur_depth, max_depth);
	}

	void validateConnectivity() const override;

	// Containers derive their required interface from their children
	// UNKNOWN until resolveInterface was called
	InterfaceFlags requiredInterface() const override { return required_interface_; }

	// forward these methods to the public interface for containers
	bool canCompute() const override;
	void compute() override;

	// internal interface for first/last child to push to if required
	InterfacePtr pendingBackward() const { return pending_backward_; }
	InterfacePtr pendingForward() const { return pending_forward_; }

	// tags for internal_external_ bimap
	struct INTERNAL
	{};
	struct EXTERNAL
	{};
	// map InterfaceStates from children to external InterfaceStates of the container
	inline const auto& internalToExternalMap() const { return internal_external_.by<INTERNAL>(); }
	inline const auto& externalToInternalMap() const { return internal_external_.by<EXTERNAL>(); }

	/// called by a (direct) child when a solution failed
	virtual void onNewFailure(const Stage& child, const InterfaceState* from, const InterfaceState* to);

protected:
	ContainerBasePrivate(ContainerBase* me, const std::string& name);

	// Set child's push interfaces: allow pushing if child requires it.
	inline void setChildsPushBackwardInterface(StagePrivate* child) {
		InterfaceFlags required = child->requiredInterface();
		bool allowed = (required & WRITES_PREV_END);
		child->setPrevEnds(allowed ? pending_backward_ : InterfacePtr());
	}
	inline void setChildsPushForwardInterface(StagePrivate* child) {
		InterfaceFlags required = child->requiredInterface();
		bool allowed = (required & WRITES_NEXT_START);
		child->setNextStarts(allowed ? pending_forward_ : InterfacePtr());
	}

	/// Set ENABLED / PRUNED status of the solution tree starting from s into given direction
	template <Interface::Direction dir>
	void setStatus(const InterfaceState* s, InterfaceState::Status status);

	/// copy external_state to a child's interface and remember the link in internal_external map
	template <Interface::Direction>
	void copyState(Interface::iterator external, const InterfacePtr& target, bool updated);
	/// non-template version
	void copyState(Interface::Direction dir, Interface::iterator external, const InterfacePtr& target, bool updated);
	/// lift solution from internal to external level
	void liftSolution(const SolutionBasePtr& solution, const InterfaceState* internal_from,
	                  const InterfaceState* internal_to);

	/// protected writable overloads
	inline auto& internalToExternalMap() { return internal_external_.by<INTERNAL>(); }
	inline auto& externalToInternalMap() { return internal_external_.by<EXTERNAL>(); }

	// set in resolveInterface()
	InterfaceFlags required_interface_;

private:
	container_type children_;

	// map start/end states of children (internal) to corresponding states in our external interfaces
	boost::bimap<boost::bimaps::unordered_set_of<boost::bimaps::tagged<const InterfaceState*, INTERNAL>>,
	             boost::bimaps::unordered_multiset_of<boost::bimaps::tagged<const InterfaceState*, EXTERNAL>>>
	    internal_external_;

	/* TODO: these interfaces don't need to be priority-sorted.
	 * Introduce base class UnsortedInterface (which is a plain list) for this use case. */
	// interface to receive children's sendBackward() states
	InterfacePtr pending_backward_;
	// interface to receive children's sendForward() states
	InterfacePtr pending_forward_;
};
PIMPL_FUNCTIONS(ContainerBase)

/* A solution of a SerialContainer needs to connect start to end via a full path.
 * The solution of a single child stage is usually disconnected to the container's start or end.
 * Only if all the children in the chain have found a coherent solution from start to end,
 * this solution can be announced as a solution of the SerialContainer.
 */
class SerialContainerPrivate : public ContainerBasePrivate
{
	friend class SerialContainer;

public:
	SerialContainerPrivate(SerialContainer* me, const std::string& name);

	// called by parent asking for pruning of this' interface
	void resolveInterface(InterfaceFlags expected) override;
	// validate connectivity of chain
	void validateConnectivity() const override;

	void reset();

protected:
	// connect two neighbors
	void connect(StagePrivate& stage1, StagePrivate& stage2);

	// validate that child's interface matches mine (considering start or end only as determined by mask)
	template <unsigned int mask>
	void validateInterface(const StagePrivate& child, InterfaceFlags required) const;
};
PIMPL_FUNCTIONS(SerialContainer)

class ParallelContainerBasePrivate : public ContainerBasePrivate
{
	friend class ParallelContainerBase;

public:
	ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string& name);

	// called by parent asking for pruning of this' interface
	void resolveInterface(InterfaceFlags expected) override;

	void validateConnectivity() const override;

protected:
	void validateInterfaces(const StagePrivate& child, InterfaceFlags& external, bool first = false) const;

	/// notify callback for new states received on external interfaces
	template <typename Interface::Direction>
	void propagateStateToChildren(Interface::iterator external, bool updated);

private:
	// override for custom behavior on received interface states
	virtual void initializeExternalInterfaces();
};
PIMPL_FUNCTIONS(ParallelContainerBase)

class FallbacksPrivate : public ParallelContainerBasePrivate
{
	friend class Fallbacks;
	struct PendingStates;
	friend std::ostream& operator<<(std::ostream& os, const FallbacksPrivate::PendingStates& pending);

public:
	FallbacksPrivate(Fallbacks* me, const std::string& name);

protected:
	struct ExternalState
	{
		ExternalState() = default;
		ExternalState(Interface::iterator state, container_type::const_iterator child)
		  : external_state(state), stage(child) {}

		Interface::iterator external_state;
		container_type::const_iterator stage;

		inline bool operator<(const ExternalState& other) const { return *external_state < *other.external_state; }
	};

	inline void computeGenerate();
	void computeFromExternal();

	struct PendingStates
	{
		std::pair<ExternalState, Interface::Direction> pop();
		inline void push(const ExternalState& job, Interface::Direction dir);
		inline bool empty() const { return pending_states_[0].empty() && pending_states_[1].empty(); }

		/// get pending states queue for a given direction
		template <Interface::Direction dir>
		inline ordered<ExternalState>& queue() {
			static_assert(dir >= 0 && dir < 2, "Expecting dir = FORWARD | BACKWARD");
			return pending_states_[dir];
		}

		inline auto& current() { return current_; }

		ordered<ExternalState> pending_states_[2];  ///< separate queues for start / end states
		Interface::Direction current_queue_ = Interface::FORWARD;  ///< which queue to check on next pop
		struct
		{
			bool valid = false;
			ExternalState state;
			Interface::Direction dir = Interface::FORWARD;
		} current_;

	} pending_;

	bool seekToNextPending();

	container_type::const_iterator current_generator_;
	bool seekToNextGenerator();

private:
	void initializeExternalInterfaces() override;
	template <typename Interface::Direction>
	void onNewExternalState(Interface::iterator external, bool updated);
	void onNewFailure(const Stage& child, const InterfaceState* from, const InterfaceState* to) override;

	void advanceCurrentStateToNextChild();
	// print pending states for debugging
	void printPending(const char* comment = "pending: ") const;
};
PIMPL_FUNCTIONS(Fallbacks)
std::ostream& operator<<(std::ostream& os, const FallbacksPrivate::PendingStates& pending);

class WrapperBasePrivate : public ParallelContainerBasePrivate
{
	friend class WrapperBase;

public:
	WrapperBasePrivate(WrapperBase* me, const std::string& name);
};
PIMPL_FUNCTIONS(WrapperBase)

class MergerPrivate : public ParallelContainerBasePrivate
{
	friend class Merger;

	moveit::core::JointModelGroupPtr jmg_merged_;
	using ChildSolutionList = std::vector<const SubTrajectory*>;
	using ChildSolutionMap = std::map<const Stage*, ChildSolutionList>;
	// map from external source state (iterator) to all corresponding children's solutions
	std::map<const InterfaceState*, ChildSolutionMap> source_state_to_solutions_;

public:
	using Spawner = std::function<void(SubTrajectory&&)>;
	MergerPrivate(Merger* me, const std::string& name);

	void resolveInterface(InterfaceFlags expected) override;

	void onNewPropagateSolution(const SolutionBase& s);
	void onNewGeneratorSolution(const SolutionBase& s);
	void mergeAnyCombination(const ChildSolutionMap& all_solutions, const SolutionBase& current,
	                         const planning_scene::PlanningSceneConstPtr& start_scene, const Spawner& spawner);
	void merge(const ChildSolutionList& sub_solutions, const planning_scene::PlanningSceneConstPtr& start_scene,
	           const Spawner& spawner);

	void sendForward(SubTrajectory&& t, const InterfaceState* from);
	void sendBackward(SubTrajectory&& t, const InterfaceState* to);
};
PIMPL_FUNCTIONS(Merger)
}  // namespace task_constructor
}  // namespace moveit
