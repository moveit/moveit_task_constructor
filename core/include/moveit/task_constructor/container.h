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
   Desc:   Container Stages to combine multiple planning Stage in different ways
*/

#pragma once

#include "stage.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
/** Base class for all container stages, i.e. ones that have one or more children */
class ContainerBase : public Stage
{
public:
	PRIVATE_CLASS(ContainerBase)

	size_t numChildren() const;

	typedef std::function<bool(const Stage&, int depth)> StageCallback;
	/// traverse direct children of this container, calling the callback for each of them
	bool traverseChildren(const StageCallback &processor) const;
	/// traverse all children of this container recursively
	bool traverseRecursively(const StageCallback &processor) const;

	virtual bool insert(Stage::pointer&& stage, int before = -1);
	virtual bool remove(int pos);
	virtual void clear();

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr& scene) override;

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	size_t numFailures() const override { return 0; }
	void processFailures(const SolutionProcessor&) const override {}

	/// called by a (direct) child when a new solution becomes available
	virtual void onNewSolution(const SolutionBase& s) = 0;

protected:
	ContainerBase(ContainerBasePrivate* impl);
};


class SerialContainerPrivate;
/** SerialContainer allows to sequentially chain a set of child stages */
class SerialContainer : public ContainerBase
{
public:
	PRIVATE_CLASS(SerialContainer)
	SerialContainer(const std::string& name = "serial container");

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const override;
	bool compute() override;

	size_t numSolutions() const override;
	void processSolutions(const SolutionProcessor &processor) const override;

	/// container used to represent a serial solution
	typedef std::vector<const SolutionBase*> solution_container;

protected:
	/// called by a (direct) child when a new solution becomes available
	void onNewSolution(const SolutionBase &s) override;

	typedef std::function<void(const solution_container& trace,
	                           double trace_accumulated_cost)> SolutionProcessor;

	/// Traverse all solution pathes starting at start and going in given direction dir
	/// until the end, i.e. until there are no more subsolutions in the given direction
	/// For each solution path, callback the given processor passing
	/// the full trace (from start to end, but not including start) and its accumulated costs
	template<Interface::Direction dir>
	void traverse(const SolutionBase &start, const SolutionProcessor &cb,
	              solution_container &trace, double trace_cost = 0);

protected:
	SerialContainer(SerialContainerPrivate* impl);
};


class ParallelContainerBasePrivate;
class ParallelContainerBase;
/** Parallel containers allow for alternative planning stages
 *  Parallel containers can come in different flavours:
 *  - alternatives: each child stage can contribute a solution
 *  - fallbacks: the children are considered in series
 *  - merged: solutions of all children (actuating disjoint groups)
 *            are merged into a single solution for parallel execution
*/
class ParallelContainerBase : public ContainerBase
{
public:
	PRIVATE_CLASS(ParallelContainerBase)
	ParallelContainerBase(const std::string &name);

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;

protected:
	ParallelContainerBase(ParallelContainerBasePrivate* impl);

	virtual void onNewSolution(const SolutionBase& s) override;
};


/** Wrap an existing solution - for use in containers.
 *
 * This essentially wraps a solution of a child and thus allows
 * for new clones of start / end states, which in turn will
 * have separate incoming/outgoing trajectories */
class WrappedSolution : public SolutionBase {
public:
	explicit WrappedSolution(StagePrivate* creator, const SolutionBase* wrapped)
	   : SolutionBase(creator, wrapped->cost()), wrapped_(wrapped)
	{}
	void fillMessage(moveit_task_constructor_msgs::Solution &solution,
                    Introspection* introspection = nullptr) const override {
		wrapped_->fillMessage(solution, introspection);
	}

private:
	const SolutionBase* wrapped_;
};


/** Plan for different alternatives in parallel.
 *
 * Solution of all children are reported - sorted by cost.
 */
class Alternatives : public ParallelContainerBase
{
public:
	Alternatives(const std::string &name) : ParallelContainerBase(name) {}

	bool canCompute() const override;
	bool compute() override;
};


/** Plan for different alternatives in sequence.
 *
 * Try to find feasible solutions using first child. Only if this fails,
 * proceed to the next child trying an alternative planning strategy.
 * All solutions of the last active child are reported.
 */
class Fallbacks : public ParallelContainerBase
{
	mutable Stage* active_child_ = nullptr;

public:
	Fallbacks(const std::string &name) : ParallelContainerBase(name) {}

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const override;
	bool compute() override;
};


class WrapperBasePrivate;
/** Base class for Wrapper and Task
 *
 * WrapperBase ensures that only a single child is wrapped in a container.
 * This child can be accessed via wrapped().
 */
class WrapperBase : public ContainerBase
{
public:
	PRIVATE_CLASS(WrapperBase)
	WrapperBase(const std::string &name, pointer &&child = Stage::pointer());

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;

	size_t numSolutions() const override;

	/// insertion is only allowed if children() is empty
	bool insert(Stage::pointer&& stage, int before = -1) override;

	/// access the single wrapped child
	Stage* wrapped();
	inline const Stage* wrapped() const {
		return const_cast<WrapperBase*>(this)->wrapped();
	}

protected:
	virtual void onNewSolution(const SolutionBase& s) = 0;

	WrapperBase(WrapperBasePrivate *impl, pointer &&child = Stage::pointer());
};


class WrapperPrivate;
/** A wrapper wraps a single generator-style stage (and acts itself as a generator).
 *
 * The wrapped stage must act as a generator, i.e. only spawn new states
 * in its external interfaces. It's intended, e.g. to filter or clone
 * a generated solution of its wrapped generator.
 */
class Wrapper : public WrapperBase
{
public:
	PRIVATE_CLASS(Wrapper)
	Wrapper(const std::string &name, pointer &&child = Stage::pointer());

	void reset() override;
	bool canCompute() const override;
	bool compute() override;

	size_t numSolutions() const override;
	size_t numFailures() const override;
	void processSolutions(const SolutionProcessor &processor) const override;
	void processFailures(const SolutionProcessor &processor) const override;

	void spawn(InterfaceState &&state, std::unique_ptr<SolutionBase>&& s);
	void spawn(InterfaceState &&state, double cost) {
		std::unique_ptr<SolutionBase> s(new SubTrajectory());
		s->setCost(cost);
		spawn(std::move(state), std::move(s));
	}

protected:
	Wrapper(WrapperPrivate* impl, pointer &&child = Stage::pointer());

	/// called by a (direct) child when a new solution becomes available
	void onNewSolution(const SolutionBase &s) override = 0;
};

} }
