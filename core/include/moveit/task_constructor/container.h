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

namespace moveit {
namespace task_constructor {

class ContainerBasePrivate;
/** Base class for all container stages, i.e. ones that have one or more children */
class ContainerBase : public Stage
{
public:
	PRIVATE_CLASS(ContainerBase)
	typedef std::unique_ptr<ContainerBase> pointer;

	size_t numChildren() const;
	Stage* findChild(const std::string& name) const;

	typedef std::function<bool(const Stage&, int depth)> StageCallback;
	/// traverse direct children of this container, calling the callback for each of them
	bool traverseChildren(const StageCallback& processor) const;
	/// traverse all children of this container recursively
	bool traverseRecursively(const StageCallback& processor) const;

	virtual bool insert(Stage::pointer&& stage, int before = -1);
	virtual bool remove(int pos);
	virtual bool remove(Stage* child);
	virtual void clear();

	void reset() override;
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	virtual bool canCompute() const = 0;
	virtual void compute() = 0;

	/// called by a (direct) child when a new solution becomes available
	virtual void onNewSolution(const SolutionBase& s) = 0;

protected:
	ContainerBase(ContainerBasePrivate* impl);
};
std::ostream& operator<<(std::ostream& os, const ContainerBase& stage);

class SerialContainerPrivate;
/** SerialContainer allows to sequentially chain a set of child stages */
class SerialContainer : public ContainerBase
{
public:
	PRIVATE_CLASS(SerialContainer)
	SerialContainer(const std::string& name = "serial container");

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	bool canCompute() const override;
	void compute() override;

protected:
	/// called by a (direct) child when a new solution becomes available
	void onNewSolution(const SolutionBase& s) override;

	typedef std::function<void(const SolutionSequence::container_type& trace, double trace_accumulated_cost)>
	    SolutionProcessor;

	/// Traverse all solution pathes starting at start and going in given direction dir
	/// until the end, i.e. until there are no more subsolutions in the given direction
	/// For each solution path, callback the given processor passing
	/// the full trace (from start to end, but not including start) and its accumulated costs
	template <Interface::Direction dir>
	void traverse(const SolutionBase& start, const SolutionProcessor& cb, SolutionSequence::container_type& trace,
	              double trace_cost = 0);

protected:
	SerialContainer(SerialContainerPrivate* impl);
};

class ParallelContainerBasePrivate;
class ParallelContainerBase;
/** Parallel containers allow for alternative planning stages
 *  Parallel containers can come in different flavours:
 *  - Alternatives: each child stage can contribute a solution
 *  - Fallbacks: the children are considered in series
 *  - Merger: solutions of all children (actuating disjoint groups)
 *            are merged into a single solution for parallel execution
*/
class ParallelContainerBase : public ContainerBase
{
public:
	PRIVATE_CLASS(ParallelContainerBase)
	ParallelContainerBase(const std::string& name = "parallel container");

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

protected:
	ParallelContainerBase(ParallelContainerBasePrivate* impl);

	/// lift unmodified child solution (useful for simple filtering)
	inline void liftSolution(const SolutionBase& solution) { liftSolution(solution, solution.cost()); }
	/// lift child solution to external interface, adapting the costs
	void liftSolution(const SolutionBase& solution, double cost) { liftSolution(solution, cost, solution.comment()); }

	/// lift child solution to external interface, adapting the costs and comment
	void liftSolution(const SolutionBase& solution, double cost, std::string comment);

	/// spawn a new solution with given state as start and end
	void spawn(InterfaceState&& state, SubTrajectory&& trajectory);
	/// propagate a solution forwards
	void sendForward(const InterfaceState& from, InterfaceState&& to, SubTrajectory&& trajectory);
	/// propagate a solution backwards
	void sendBackward(InterfaceState&& from, const InterfaceState& to, SubTrajectory&& trajectory);
};

/** Plan for different alternatives in parallel.
 *
 * Solution of all children are reported - sorted by cost.
 */
class Alternatives : public ParallelContainerBase
{
public:
	Alternatives(const std::string& name = "alternatives") : ParallelContainerBase(name) {}

	bool canCompute() const override;
	void compute() override;

	void onNewSolution(const SolutionBase& s) override;
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
	Fallbacks(const std::string& name = "fallbacks") : ParallelContainerBase(name) {}

	void reset() override;
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;

	void onNewSolution(const SolutionBase& s) override;
};

class MergerPrivate;
/** Plan for different sub tasks in parallel and finally merge all sub solutions into a single trajectory */
class Merger : public ParallelContainerBase
{
public:
	PRIVATE_CLASS(Merger)
	Merger(const std::string& name = "merger");

	void reset() override;
	void init(const core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;

protected:
	Merger(MergerPrivate* impl);
	void onNewSolution(const SolutionBase& s) override;
};

class WrapperBasePrivate;
/** A wrapper wraps a single child stage, which can be accessed via wrapped().
 *
 * Implementations of this interface need to implement onNewSolution(), which is
 * called when the child has generated a new solution.
 * The wrapper may reject the solution or create one or multiple derived solutions,
 * potentially adapting the cost, as well as its start and end states.
 */
class WrapperBase : public ParallelContainerBase
{
public:
	PRIVATE_CLASS(WrapperBase)
	WrapperBase(const std::string& name = "wrapper", Stage::pointer&& child = Stage::pointer());

	/// insertion is only allowed if children() is empty
	bool insert(Stage::pointer&& stage, int before = -1) override;

	/// access the single wrapped child, NULL if still empty
	Stage* wrapped();
	inline const Stage* wrapped() const { return const_cast<WrapperBase*>(this)->wrapped(); }

	bool canCompute() const override;
	void compute() override;

protected:
	WrapperBase(WrapperBasePrivate* impl, Stage::pointer&& child = Stage::pointer());
};
}
}
