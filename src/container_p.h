#pragma once

#include <moveit_task_constructor/container.h>
#include "subtask_p.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate : public SubTaskPrivate
{
	friend class BaseTest; // allow access for unit tests
public:
	typedef ContainerBase::value_type value_type;
	typedef SubTaskPrivate::array_type array_type;
	typedef array_type::iterator iterator;
	typedef array_type::const_iterator const_iterator;

	inline const array_type& children() const { return children_; }
	const_iterator position(int before = -1) const;

	bool canInsert(const SubTask& stage) const;
	virtual iterator insert(value_type &&subtask, const_iterator pos);
	inline void clear() { children_.clear(); }

	bool traverseStages(const ContainerBase::StageCallback &processor, int depth) const;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{}
	inline const ContainerBasePrivate* parent(const SubTaskPrivate *child) const { return child->parent_; }
	inline iterator it(const SubTaskPrivate *child) const { return child->it_; }

	inline void setPrevOutput(const SubTaskPrivate* child, const InterfacePtr& interface = InterfacePtr()) {
		child->prev_output_ = interface.get();
	}
	inline void setNextInput(const SubTaskPrivate* child, const InterfacePtr& interface = InterfacePtr()) {
		child->next_input_ = interface.get();
	}

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

	SubTask::InterfaceFlags announcedFlags() const override;
	bool canInsert(const SubTask& stage, const_iterator before) const;
	virtual iterator insert(value_type &&stage, const_iterator before) override;

	inline const SubTaskPrivate *prev(const_iterator it) const;
	inline const SubTaskPrivate *next(const_iterator it) const;

	const SubTaskPrivate *prev(const SubTaskPrivate *child) const;
	const SubTaskPrivate *next(const SubTaskPrivate *child) const;
};


} }
