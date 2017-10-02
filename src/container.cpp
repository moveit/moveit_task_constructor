#include "container_p.h"

#include <memory>
#include <iostream>

namespace moveit { namespace task_constructor {

ContainerBasePrivate::const_iterator ContainerBasePrivate::position(int before) const {
	const_iterator position = children_.begin();
	if (before > 0) {
		for (auto end = children_.end(); before > 0 && position != end; --before)
			++position;
	} else if (++before <= 0) {
		array_type::const_reverse_iterator from_end = children_.rbegin();
		for (auto end = children_.rend(); before < 0 && from_end != end; ++before)
			++from_end;
		position = from_end.base();
	}
	return position;
}

bool ContainerBasePrivate::canInsert(const SubTask &stage) const {
	const SubTaskPrivate* impl = stage.pimpl_func();
	return impl->parent_ == nullptr  // re-parenting is not supported
	      && impl->predeccessor_ == nullptr
	      && impl->successor_ == nullptr
	      && impl->trajectories_.empty(); // existing trajectories would become invalid
}

// insert child into chain, before where
void ContainerBasePrivate::insertSerial(ContainerBasePrivate::value_type &&child, ContainerBasePrivate::const_iterator before) {
	SubTaskPrivate* child_impl = child->pimpl_func();
	bool at_end = before == children_.cend();
	bool at_begin = before == children_.cbegin();

	// child should not be connected yet
	assert(child_impl->parent_ == nullptr);
	assert(child_impl->predeccessor_ == nullptr);
	assert(child_impl->successor_ == nullptr);

	child_impl->parent_ = this;
	if (children_.empty()) {
		child_impl->successor_ = this;
		child_impl->predeccessor_ = this;
	} else if (at_end) {
		const_iterator prev = before; --prev;
		SubTaskPrivate* prev_impl = prev->get()->pimpl_func();
		child_impl->successor_ = this;
		child_impl->predeccessor_ = prev_impl;
		prev_impl->successor_ = child_impl;
	} else if (at_begin) {
		SubTaskPrivate* next_impl = before->get()->pimpl_func();
		child_impl->predeccessor_ = this;
		child_impl->successor_ = next_impl;
		next_impl->predeccessor_ = child_impl;
	} else {
		const_iterator prev = before; --prev;
		SubTaskPrivate* prev_impl = prev->get()->pimpl_func();
		SubTaskPrivate* next_impl = before->get()->pimpl_func();
		child_impl->successor_ = next_impl;
		child_impl->predeccessor_ = prev_impl;
		next_impl->predeccessor_ = child_impl;
		prev_impl->successor_ = child_impl;
	}
	ContainerBasePrivate::insert(std::move(child), before);
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor, int depth) const {
	for (auto &stage : children_) {
		if (!processor(*stage, depth))
			continue;
		ContainerBase *container = dynamic_cast<ContainerBase*>(stage.get());
		if (container)
			container->pimpl_func()->traverseStages(processor, depth+1);
	}
	return true;
}

ContainerBasePrivate::iterator ContainerBasePrivate::insert(ContainerBasePrivate::value_type &&subtask, ContainerBasePrivate::const_iterator pos) {
	subtask->setPlanningScene(scene_);
	subtask->setPlanningPipeline(planner_);
	return children_.insert(pos, std::move(subtask));
}


PRIVATE_CLASS_IMPL(ContainerBase)
ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : SubTask(impl)
{
}

void ContainerBase::clear()
{
	IMPL(ContainerBase);
	impl->clear();
}

bool ContainerBase::traverseStages(const ContainerBase::StageCallback &processor) const
{
	IMPL(const ContainerBase);
	return impl->traverseStages(processor, 0);
}


bool SerialContainerPrivate::canInsert(const ContainerBasePrivate::value_type &subtask, ContainerBasePrivate::const_iterator before) const {
	return ContainerBasePrivate::canInsert(*subtask);
}


PRIVATE_CLASS_IMPL(SerialContainer)
SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}

bool SerialContainer::canInsert(const value_type& subtask, int before) const
{
	IMPL(const SerialContainer);
	return impl->canInsert(subtask, impl->position(before));
}

bool SerialContainer::insert(value_type&& subtask, int before)
{
	IMPL(SerialContainer);

	ContainerBasePrivate::const_iterator where = impl->position(before);
	if (!impl->canInsert(subtask, where))
		return false;

	impl->insertSerial(std::move(subtask), where);
	return true;
}

bool SerialContainer::canCompute() const
{
	IMPL(const SerialContainer);
	return impl->children_.size() > 0;
}

bool SerialContainer::compute()
{
	IMPL(SerialContainer);
	bool computed = false;
	for(const auto& stage : impl->children_){
		if(!stage->canCompute())
			continue;
		std::cout << "Computing subtask '" << stage->getName() << "':" << std::endl;
		bool success = stage->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
	}
	return computed;
}

} }
