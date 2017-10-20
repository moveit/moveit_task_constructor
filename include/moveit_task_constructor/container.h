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
	virtual void clear();

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr& scene) override;

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	size_t numSolutions() const = 0;
	typedef std::function<bool(const SolutionBase&)> SolutionProcessor;
	/// process all solutions, calling the callback for each of them
	virtual void processSolutions(const SolutionProcessor &processor) const = 0;

protected:
	ContainerBase(ContainerBasePrivate* impl);
};


class SerialContainerPrivate;
/** SerialContainer allows to sequentially chain a set of child stages */
class SerialContainer : public ContainerBase
{
public:
	PRIVATE_CLASS(SerialContainer)
	SerialContainer(const std::string& name);

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const override;
	bool compute() override;

	size_t numSolutions() const override;
	void processSolutions(const SolutionProcessor &processor) const;

protected:
	/// function type used for traversing solutions
	/// For each sub solution (current), the trace from the start as well as the
	/// accumulated cost of all solutions in the trace are provided.
	/// Return true, if traversal should continue, otherwise false.
	typedef std::function<bool(const SolutionBase& current,
	                           const std::vector<const SolutionBase*>& trace,
	                           double trace_accumulated_cost)> SolutionProcessor;

	/// traverse all solutions, starting at start and call the callback for each subsolution
	/// The return value is always false, indicating that the traversal eventually stopped.
	template<TraverseDirection dir>
	bool traverse(const SolutionBase &start, const SolutionProcessor &cb,
	              std::vector<const SolutionBase*> &trace, double trace_cost = 0);

protected:
	SerialContainer(SerialContainerPrivate* impl);
};


class WrapperBasePrivate;
/** Wrappers wrap a single child stage. WrapperBase is an abstract base class */
class WrapperBase : protected ContainerBase
{
public:
	PRIVATE_CLASS(WrapperBase)
	WrapperBase(const std::string &name, pointer &&child = Stage::pointer());
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;

protected:
	WrapperBase(WrapperBasePrivate *impl);
	// insertion is only allowed if children() is empty
	bool insert(Stage::pointer&& stage, int before = -1) override;

	/// access the single wrapped child
	Stage* wrapped();
	inline const Stage* wrapped() const {
		return const_cast<WrapperBase*>(this)->wrapped();
	}

private:
	/// append s as SubTrajectories to solution (lifted from private API)
	virtual void append(const SolutionBase& s, std::vector<const SubTrajectory*>& solution) const = 0;
	/// callback for new solutions
	virtual void onNewSolution(SolutionBase &s) = 0;
};

} }
