#pragma once

#include <moveit_task_constructor/container.h>
#include "stage_p.h"

#include <map>
#include <climits>

namespace moveit { namespace task_constructor {

class ContainerBasePrivate : public StagePrivate
{
	friend class ContainerBase;

public:
	typedef StagePrivate::container_type container_type;
	typedef container_type::iterator iterator;
	typedef container_type::const_iterator const_iterator;

	inline const container_type& children() const { return children_; }

	// Retrieve iterator into children_ pointing to indexed element.
	// Negative index counts from end().
	// Contrary to std::advance(), iterator limits are considered.
	const_iterator position(int index) const;

	// traversing all stages upto max_depth
	bool traverseStages(const ContainerBase::StageCallback &processor,
	                    unsigned int cur_depth, unsigned int max_depth) const;

	// forward these methods to the public interface for containers
	bool canCompute() const override;
	bool compute() override;

	// callback when a new trajectory or combined solution becomes available
	virtual void onNewSolution(SolutionBase& t) = 0;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : StagePrivate(me, name)
	{}

private:
	container_type children_;
};


class SerialSolution : public SolutionBase {
public:
	explicit SerialSolution(StagePrivate* creator, std::vector<const SolutionBase*>&& subsolutions, double cost)
	   : SolutionBase(creator, cost), subsolutions_(subsolutions)
	{}
	void appendTo(std::vector<const SubTrajectory*>& solution) const;

private:
	// series of sub solutions
	std::vector<const SolutionBase*> subsolutions_;
};


class SerialContainerPrivate : public ContainerBasePrivate {
	friend class SerialContainer;

public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name);

	void onNewSolution(SolutionBase &s) override;
	void storeNewSolution(std::vector<const SolutionBase *> &&s, double cost);
	const std::list<SerialSolution>& solutions() const { return solutions_; }

	void append(const SolutionBase& s, std::vector<const SubTrajectory*>& solution) const override {
		assert(s.creator() == this);
		static_cast<const SerialSolution&>(s).appendTo(solution);
	}

private:
	inline const_iterator prev(const_iterator it) const;
	inline const_iterator next(const_iterator it) const;
	void connect(StagePrivate *prev, StagePrivate *next);

	/* A container needs to decouple its interface from those of its children:
	 * A solution of a container needs to connect start to end via a full path.
	 * Start/end states of a single stage may only need to have a single outgoing/incoming trajectory.
	 * Note, that there might be many solutions connecting the same start-end state pair. */

	// map first child's start states to the corresponding states in this' starts_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_starts_;
	// map last child's end states to the corresponding states in this' ends_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_ends_;

	/* First/last childrens sendBackward()/sendForward() states are not directly propagated
	 * to previous/next stage of this container, because we cannot provide a solution yet.
	 * Only if we have full solution from start to end available, we can propagate the states */
	// interface to receive first child's sendBackward() states
	InterfacePtr pending_backward_;
	// interface to receive last child's sendForward() states
	InterfacePtr pending_forward_;

	std::list<SerialSolution> solutions_;
};

} }
