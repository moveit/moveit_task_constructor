#pragma once

#include "subtask.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
class ContainerBase : public SubTask
{
public:
	PRIVATE_CLASS(ContainerBase)
	typedef SubTask::pointer value_type;

	typedef std::function<bool(const SubTask&, int depth)> StageCallback;
	typedef std::function<bool(const std::vector<SubTrajectory*>&)> SolutionCallback;

	virtual bool canInsert(const value_type& stage, int before = -1) const = 0;
	virtual bool insert(value_type&& stage, int before = -1) = 0;
	virtual void clear();

	bool traverseStages(const StageCallback &processor) const;

protected:
	ContainerBase(ContainerBasePrivate* impl);
};

class SerialContainerPrivate;
class SerialContainer : public ContainerBase
{
public:
	PRIVATE_CLASS(SerialContainer)
	SerialContainer(const std::string& name);

	bool canInsert(const value_type& stage, int before = -1) const override;
	bool insert(value_type&& stage, int before = -1) override;

	bool canCompute() const override;
	bool compute() override;

protected:
	SerialContainer(SerialContainerPrivate* impl);
};

} }
