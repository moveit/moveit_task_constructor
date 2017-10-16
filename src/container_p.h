#pragma once

#include <moveit_task_constructor/container.h>
#include "stage_p.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate : public StagePrivate
{
	friend class ContainerBase;

public:
	typedef ContainerBase::value_type value_type;
	typedef StagePrivate::container_type container_type;
	typedef container_type::iterator iterator;
	typedef container_type::const_iterator const_iterator;

	inline const container_type& children() const { return children_; }
	const_iterator position(int before = -1) const;

	bool canInsert(const Stage& stage) const;
	virtual iterator insert(value_type &&stage, const_iterator pos);

	bool traverseStages(const ContainerBase::StageCallback &processor, int depth) const;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : StagePrivate(me, name)
	{}

private:
	container_type children_;
};


class SerialContainerPrivate : public ContainerBasePrivate {
public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name);

	InterfaceFlags announcedFlags() const override;
	inline bool canInsert(const Stage& stage, const_iterator before) const;
	virtual iterator insert(value_type &&stage, const_iterator before) override;

	bool canCompute() const override;
	bool compute() override;

private:
	inline const_iterator prev(const_iterator it) const;
	inline const_iterator next(const_iterator it) const;
};


} }
