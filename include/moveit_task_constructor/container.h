#pragma once

#include "stage.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
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
	virtual void clear();

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	size_t numSolutions() const = 0;

protected:
	ContainerBase(ContainerBasePrivate* impl);
};

class SerialContainerPrivate;
class SerialContainer : public ContainerBase
{
public:
	PRIVATE_CLASS(SerialContainer)
	SerialContainer(const std::string& name);

	void init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const override;
	bool compute() override;

	size_t numSolutions() const override;

	/// function type used for traversing solutions
	/// For each sub solution (current), the trace from the start as well as the
	/// accumulated cost of all solutions in the trace are provided.
	/// Return true, if traversal should continue, otherwise false.
	typedef std::function<bool(const SolutionBase& current,
	                           const std::vector<const SolutionBase*>& trace,
	                           double trace_accumulated_cost)> SolutionCallback;

protected:
	/// traverse all solutions, starting at start and call the callback for each subsolution
	/// The return value is always false, indicating that the traversal eventually stopped.
	template<TraverseDirection dir>
	bool traverse(const SolutionBase &start, const SolutionCallback &cb,
	              std::vector<const SolutionBase*> &trace, double trace_cost = 0);

protected:
	SerialContainer(SerialContainerPrivate* impl);
};

} }
