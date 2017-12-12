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

	/// called by a (direct) child when a new solution becomes available
	virtual void onNewSolution(SolutionBase& t) = 0;

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
	void onNewSolution(SolutionBase &s) override;

	/// function type used for traversing solutions
	/// For each sub solution (current), the trace from the start as well as the
	/// accumulated cost of all solutions in the trace are provided.
	/// Return true, if traversal should continue, otherwise false.
	typedef std::function<bool(const SolutionBase& current,
	                           const solution_container& trace,
	                           double trace_accumulated_cost)> SolutionProcessor;

	/// traverse all solutions, starting at start and call the callback for each subsolution
	/// The return value is always false, indicating that the traversal eventually stopped.
	template<TraverseDirection dir>
	bool traverse(const SolutionBase &start, const SolutionProcessor &cb,
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

	size_t numSolutions() const override;
	void processSolutions(const SolutionProcessor &processor) const override;

protected:
	ParallelContainerBase(ParallelContainerBasePrivate* impl);

	/// called by a (direct) child when a new solution becomes available
	void onNewSolution(SolutionBase &s) override;

	/// callback for new start states (received externally)
	virtual void onNewStartState(const InterfaceState &external) = 0;
	/// callback for new end states (received externally)
	virtual void onNewEndState(const InterfaceState &external) = 0;
};


/** A wrapper can wrap a single generator-style stage (and acts itself as a generator).
 *
 * The wrapped stage must act as a generator, i.e. only spawn new states
 * in its external interfaces. It's intended, e.g. to filter or clone
 * a generated solution of its wrapped generator. */
class WrapperBase : protected ParallelContainerBase
{
public:
	WrapperBase(const std::string &name, pointer &&child = Stage::pointer());
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;

protected:
	/// insertion is only allowed if children() is empty
	bool insert(Stage::pointer&& stage, int before = -1) override;

	/// access the single wrapped child
	Stage* wrapped();
	inline const Stage* wrapped() const {
		return const_cast<WrapperBase*>(this)->wrapped();
	}

private:
	void onNewStartState(const InterfaceState&) override {}
	void onNewEndState(const InterfaceState&) override {}
};

} }
