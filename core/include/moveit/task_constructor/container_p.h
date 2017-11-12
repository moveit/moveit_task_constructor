#pragma once

#include <moveit/task_constructor/container.h>
#include "stage_p.h"

#include <map>
#include <climits>

namespace moveit { namespace task_constructor {

/* A container needs to decouple its interfaces from those of its children.
 * Both, the container and the children have their own starts_ and ends_.
 * The container needs to forward states received in its interfaces to
 * the interfaces of the children.
 * Solutions found by the children, then need to be connected to the
 * container's interface states. To this end, we remember the mapping
 * from internal to external states.
 * Note, that there might be many solutions connecting a signle start-end pair.
 * These solutions might origin from different children (ParallelContainer)
 * or from different solution paths in a SerialContainer.
 */
class ContainerBasePrivate : public StagePrivate
{
	friend class ContainerBase;

public:
	typedef StagePrivate::container_type container_type;
	typedef container_type::iterator iterator;
	typedef container_type::const_iterator const_iterator;

	inline const container_type& children() const { return children_; }

	/** Retrieve iterator into children_ pointing to indexed element.
	 * Negative index counts from end().
	 * Contrary to std::advance(), iterator limits are considered. */
	const_iterator position(int index) const;

	/// traversing all stages upto max_depth
	bool traverseStages(const ContainerBase::StageCallback &processor,
	                    unsigned int cur_depth, unsigned int max_depth) const;

	// forward these methods to the public interface for containers
	bool canCompute() const override;
	bool compute() override;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : StagePrivate(me, name)
	{}
	/// copy external_state to a child's interface and remember the link in internal_to map
	void copyState(InterfaceState &external_state, Stage &child, bool to_start);

protected:
	container_type children_;

	// map first child's start states to the corresponding states in this' starts_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_starts_;
	// map last child's end states to the corresponding states in this' ends_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_ends_;
};
PIMPL_FUNCTIONS(ContainerBase)

/** Representation of a single, full solution path of a SerialContainer.
 *
 * A serial solution describes a full solution path through all children
 * of a SerialContainer. This is a vector (of children().size()) of pointers
 * to all solutions of the children. Hence, we don't need to copy those solutions. */
class SerialSolution : public SolutionBase {
public:
	explicit SerialSolution(StagePrivate* creator, SerialContainer::solution_container&& subsolutions, double cost)
	   : SolutionBase(creator, cost), subsolutions_(subsolutions)
	{}
	/// append all subsolutions to solution
	void fillMessage(moveit_task_constructor_msgs::Solution &msg, Introspection *introspection) const override;

private:
	/// series of sub solutions
	SerialContainer::solution_container subsolutions_;
};


/* A solution of a SerialContainer needs to connect start to end via a full path.
 * The solution of a single child stage is usually disconnected to the container's start or end.
 * Only if all the children in the chain have found a coherent solution from start to end,
 * this solution can be announced as a solution of the SerialContainer.
 *
 * Particularly, the first/last stage's sendBackward()/sendForward() call
 * cannot directly propagate their associated state to the previous/next stage of this container,
 * because we cannot provide a full solution yet. Hence, the first/last stage
 * propagate to the pending_backward_/pending_forward_ interface first.
 * If eventually a full solution is found, it is propagated to prevEnds()/nextStarts() -
 * together with the solution. */
class SerialContainerPrivate : public ContainerBasePrivate {
	friend class SerialContainer;

public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name);

	void storeNewSolution(SerialContainer::solution_container &&s, double cost);
	const std::list<SerialSolution>& solutions() const { return solutions_; }

private:
	void connect(StagePrivate *prev, StagePrivate *next);

	// interface to buffer first child's sendBackward() states
	InterfacePtr pending_backward_;
	// interface to buffer last child's sendForward() states
	InterfacePtr pending_forward_;

	// set of all solutions
	std::list<SerialSolution> solutions_;
};
PIMPL_FUNCTIONS(SerialContainer)


/** Representation of a single solution of a ParallelContainer.
 *
 * This essentially wraps a solution of a child and thus allows
 * for new new clones of start / end states, which in turn will
 * have separate incoming/outgoing trajectories */
class WrappedSolution : public SolutionBase {
public:
	explicit WrappedSolution(StagePrivate* creator, SolutionBase* wrapped)
	   : SolutionBase(creator, wrapped->cost()), wrapped_(wrapped)
	{}
	void fillMessage(moveit_task_constructor_msgs::Solution &solution,
                    Introspection* introspection = nullptr) const override {
		wrapped_->fillMessage(solution, introspection);
	}

private:
	SolutionBase* wrapped_;
};


class ParallelContainerBasePrivate : public ContainerBasePrivate {
	friend class ParallelContainerBase;

public:
	ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string &name);
	const std::list<WrappedSolution>& solutions() const { return solutions_; }

private:
	std::list<WrappedSolution> solutions_;
};
PIMPL_FUNCTIONS(ParallelContainerBase)

} }
