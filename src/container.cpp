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
		container_type::const_reverse_iterator from_end = children_.rbegin();
		for (auto end = children_.rend(); before < 0 && from_end != end; ++before)
			++from_end;
		position = from_end.base();
	}
	return position;
}

inline bool ContainerBasePrivate::canInsert(const Stage &stage) const {
	const StagePrivate* impl = stage.pimpl();
	return impl->parent() == nullptr  // re-parenting is not supported
	      && impl->trajectories().empty(); // existing trajectories would become invalid
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor, int depth) const {
	for (auto &stage : children_) {
		if (!processor(*stage, depth))
			continue;
		ContainerBasePrivate *container = dynamic_cast<ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, depth+1);
	}
	return true;
}

ContainerBasePrivate::iterator ContainerBasePrivate::insert(ContainerBasePrivate::value_type &&stage,
                                                            ContainerBasePrivate::const_iterator pos) {
	StagePrivate *impl = stage->pimpl();
	ContainerBasePrivate::iterator it = children_.insert(pos, std::move(stage));
	impl->setHierarchy(this, it);
	return it;
}


ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : Stage(impl)
{
}
PIMPL_FUNCTIONS(ContainerBase)

void ContainerBase::clear()
{
	pimpl()->children_.clear();
}

bool ContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	auto impl = pimpl();
	for (auto& stage : impl->children_)
		stage->init(scene);
}

bool ContainerBase::traverseStages(const ContainerBase::StageCallback &processor) const
{
	pimpl()->traverseStages(processor, 0);
}

bool ContainerBase::canCompute() const
{
	pimpl()->canCompute();
}

bool ContainerBase::compute() {
	pimpl()->compute();
}


SerialContainerPrivate::SerialContainerPrivate(SerialContainer *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	starts_.reset(new Interface(Interface::NotifyFunction()));
	ends_.reset(new Interface(Interface::NotifyFunction()));
}

StagePrivate::InterfaceFlags SerialContainerPrivate::announcedFlags() const {
	InterfaceFlags f;
	if (children().empty()) return f;
	f |= children().front()->pimpl()->announcedFlags() & INPUT_IF_MASK;
	f |= children().back()->pimpl()->announcedFlags() & OUTPUT_IF_MASK;
	return f;
}

inline bool SerialContainerPrivate::canInsert(const Stage &stage, ContainerBasePrivate::const_iterator before) const {
	if (!ContainerBasePrivate::canInsert(stage))
		return false;
	return true;
}

ContainerBasePrivate::iterator SerialContainerPrivate::insert(value_type &&stage, const_iterator before)
{
	assert(canInsert(*stage, before));
	bool at_begin = (before == children().begin());
	bool at_end = (before == children().end());

	StagePrivate *cur = stage->pimpl();
	/* set pointer cache (prev_ends_ and next_starts_) of prev, current, and next stage */
	if (children().empty()) { // first child inserted
		cur->setPrevEnds(this->starts());
		cur->setNextStarts(this->ends());
	} else if (at_begin) {
		StagePrivate *next = (*before)->pimpl();
		cur->setPrevEnds(this->starts());
		cur->setNextStarts(next->starts());
		next->setPrevEnds(cur->ends());
	} else if (at_end) {
		StagePrivate *prev = (*this->prev(before))->pimpl();
		prev->setNextStarts(cur->starts());
		cur->setPrevEnds(prev->ends());
		cur->setNextStarts(this->ends());
	} else {
		StagePrivate *prev = (*this->prev(before))->pimpl();
		StagePrivate *next = (*before)->pimpl();
		prev->setNextStarts(cur->starts());
		cur->setPrevEnds(prev->ends());
		cur->setNextStarts(next->starts());
		next->setPrevEnds(cur->ends());
	}

	iterator it = ContainerBasePrivate::insert(std::move(stage), before);
}

inline ContainerBasePrivate::const_iterator SerialContainerPrivate::prev(const_iterator it) const
{
	assert(it != children().cbegin());
	return --it;
}

inline ContainerBasePrivate::const_iterator SerialContainerPrivate::next(const_iterator it) const
{
	assert(it != children().cend());
	return ++it;
}

SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}
PIMPL_FUNCTIONS(SerialContainer)

bool SerialContainer::canInsert(const value_type& stage, int before) const
{
	auto impl = pimpl();
	return impl->canInsert(*stage, impl->position(before));
}

bool SerialContainer::insert(value_type&& stage, int before)
{
	auto impl = pimpl();

	ContainerBasePrivate::const_iterator where = impl->position(before);
	if (!impl->canInsert(*stage, where))
		return false;

	impl->insert(std::move(stage), where);
	return true;
}

bool SerialContainerPrivate::canCompute() const
{
	return children().size() > 0;
}

bool SerialContainerPrivate::compute()
{
	bool computed = false;
	for(const auto& stage : children()) {
		if(!stage->pimpl()->canCompute())
			continue;
		std::cout << "Computing stage '" << stage->getName() << "':" << std::endl;
		bool success = stage->pimpl()->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
	}
	return computed;
}

} }
