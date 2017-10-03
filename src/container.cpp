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
	      && impl->trajectories_.empty(); // existing trajectories would become invalid
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor, int depth) const {
	for (auto &stage : children_) {
		if (!processor(*stage, depth))
			continue;
		ContainerBase *container = dynamic_cast<ContainerBase*>(stage.get());
		if (container)
			static_cast<ContainerBasePrivate*>(container->pimpl_func())->traverseStages(processor, depth+1);
	}
	return true;
}

ContainerBasePrivate::iterator ContainerBasePrivate::insert(ContainerBasePrivate::value_type &&subtask,
                                                            ContainerBasePrivate::const_iterator pos) {
	SubTaskPrivate *impl = subtask->pimpl_func();
	impl->parent_ = this;
	subtask->setPlanningScene(scene_);
	subtask->setPlanningPipeline(planner_);
	impl->it_ = children_.insert(pos, std::move(subtask));
	return impl->it_;
}


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


SubTask::InterfaceFlags SerialContainerPrivate::interfaceFlags() const
{
	SubTask::InterfaceFlags f;
	if (!children().size()) return f;
	f |= children().front()->pimpl_func()->deducedInterfaceFlags() & SubTask::INPUT_IF_MASK;
	f |= children().back()->pimpl_func()->deducedInterfaceFlags() & SubTask::OUTPUT_IF_MASK;
	return f;
}

bool SerialContainerPrivate::canInsert(const ContainerBasePrivate::value_type &subtask, ContainerBasePrivate::const_iterator before) const {
	return ContainerBasePrivate::canInsert(*subtask);
}

const SubTaskPrivate *SerialContainerPrivate::prev_(const SubTaskPrivate *child) const
{
	assert(parent(child) == this);
	if (it(child) == children().begin()) return this;
	iterator prev = it(child); --prev;
	return (*prev)->pimpl_func();
}

const SubTaskPrivate *SerialContainerPrivate::next_(const SubTaskPrivate *child) const
{
	assert(parent(child) == this);
	if (it(child) == --children().end()) return this;
	iterator next = it(child); ++next;
	return (*next)->pimpl_func();
}


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

	impl->insert(std::move(subtask), where);
	return true;
}

bool SerialContainer::canCompute() const
{
	IMPL(const SerialContainer);
	return impl->children().size() > 0;
}

bool SerialContainer::compute()
{
	IMPL(SerialContainer);
	bool computed = false;
	for(const auto& stage : impl->children()){
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
