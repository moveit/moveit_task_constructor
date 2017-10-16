#pragma once

#include "stage.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
class ContainerBase : public Stage
{
public:
	PRIVATE_CLASS(ContainerBase)
	typedef Stage::pointer value_type;

	typedef std::function<bool(const Stage&, int depth)> StageCallback;
	typedef std::function<bool(const std::vector<SubTrajectory*>&)> SolutionCallback;

	virtual bool canInsert(const value_type& stage, int before = -1) const = 0;
	virtual bool insert(value_type&& stage, int before = -1) = 0;
	virtual void clear();

	bool init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const;
	bool compute();

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

protected:
	SerialContainer(SerialContainerPrivate* impl);
};

} }
