#pragma once

#include <moveit_task_constructor/container.h>
#include "subtask_p.h"

namespace moveit { namespace task_constructor {

class ContainerBasePrivate : public SubTaskPrivate
{
public:
	typedef ContainerBase::value_type value_type;
	typedef std::vector<value_type> array_type;
	typedef array_type::iterator iterator;
	typedef array_type::const_iterator const_iterator;

	array_type children_;

	const_iterator position(int before = -1) const;

	bool canInsert(const SubTask& stage) const;

	/* SerialContainer doesn't have own input_, output_ interfaces,
	 * but share the interface pointer with their first resp. last child stage.
	 * In this fashion, spawned states directly get propagated to the actual stage.
	 * Consequently, when the container is empty, both interface pointers are invalid. */
	void insertSerial(value_type&& child, const_iterator before);

	void clear() {
		children_.clear();
	}

	bool traverseStages(const ContainerBase::StageCallback &processor, int depth) const;

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{}

private:
	iterator insert(value_type &&subtask, const_iterator pos);
};


class SerialContainerPrivate : public ContainerBasePrivate {
public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name)
	   : ContainerBasePrivate(me, name)
	{
		input_.reset(new Interface(Interface::NotifyFunction()));
		output_.reset(new Interface(Interface::NotifyFunction()));
	}

	bool canInsert(const value_type& subtask, const_iterator before) const;
};


} }
