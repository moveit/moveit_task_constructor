#pragma once

#include <moveit_task_constructor/container.h>
#include "subtask_p.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate : public SubTaskPrivate
{
public:
	typedef ContainerBase::value_type value_type;
	typedef SubTaskPrivate::array_type array_type;
	typedef array_type::iterator iterator;
	typedef array_type::const_iterator const_iterator;

	inline const array_type& children() const { return children_; }
	const_iterator position(int before = -1) const;

	bool canInsert(const SubTask& stage) const;
	iterator insert(value_type &&subtask, const_iterator pos);
	inline void clear() { children_.clear(); }

	virtual const SubTaskPrivate* prev_(const SubTaskPrivate* child) const = 0;
	virtual const SubTaskPrivate* next_(const SubTaskPrivate* child) const = 0;

	bool traverseStages(const ContainerBase::StageCallback &processor, int depth) const;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{}
	inline const ContainerBasePrivate* parent(const SubTaskPrivate *child) const { return child->parent_; }
	inline iterator it(const SubTaskPrivate *child) const { return child->it_; }

private:
	array_type children_;
};


class SerialContainerPrivate : public ContainerBasePrivate {
public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name)
	   : ContainerBasePrivate(me, name)
	{
		input_.reset(new Interface(Interface::NotifyFunction()));
		output_.reset(new Interface(Interface::NotifyFunction()));
	}

	SubTask::InterfaceFlags interfaceFlags() const override;
	bool canInsert(const value_type& subtask, const_iterator before) const;

	const SubTaskPrivate* prev_(const SubTaskPrivate* child) const override;
	const SubTaskPrivate* next_(const SubTaskPrivate* child) const override;
};


} }
