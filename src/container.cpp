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

inline bool ContainerBasePrivate::canInsert(const SubTask &stage) const {
	const SubTaskPrivate* impl = stage.pimpl_func();
	return impl->parent_ == nullptr  // re-parenting is not supported
	      && impl->trajectories_.empty(); // existing trajectories would become invalid
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor, int depth) const {
	for (auto &stage : children_) {
		if (!processor(*stage, depth))
			continue;
		ContainerBasePrivate *container = dynamic_cast<ContainerBasePrivate*>(stage->pimpl_func());
		if (container)
			container->traverseStages(processor, depth+1);
	}
	return true;
}

ContainerBasePrivate::iterator ContainerBasePrivate::insert(ContainerBasePrivate::value_type &&subtask,
                                                            ContainerBasePrivate::const_iterator pos) {
	SubTaskPrivate *impl = subtask->pimpl_func();
	impl->parent_ = this;
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

bool ContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	IMPL(ContainerBase);
	for (auto& stage : impl->children_)
		stage->init(scene);
}

bool ContainerBase::traverseStages(const ContainerBase::StageCallback &processor) const
{
	IMPL(const ContainerBase);
	return impl->traverseStages(processor, 0);
}

bool ContainerBase::canCompute() const
{
	IMPL(const ContainerBase);
	return impl->canCompute();
}

bool ContainerBase::compute() {
	IMPL(ContainerBase);
	return impl->compute();
}


SubTaskPrivate::InterfaceFlags SerialContainerPrivate::announcedFlags() const {
	InterfaceFlags f;
	if (children().empty()) return f;
	f |= children().front()->pimpl_func()->announcedFlags() & INPUT_IF_MASK;
	f |= children().back()->pimpl_func()->announcedFlags() & OUTPUT_IF_MASK;
	return f;
}

inline bool isConnectable(int prev_flags, int next_flags) {
	return ((prev_flags & SubTaskPrivate::WRITES_NEXT_START) && (next_flags & SubTaskPrivate::READS_START)) ||
	       ((prev_flags & SubTaskPrivate::READS_END) && (next_flags & SubTaskPrivate::WRITES_PREV_END));
}
inline bool bothWrite(SubTaskPrivate::InterfaceFlags prev_flags, SubTaskPrivate::InterfaceFlags next_flags) {
	return (prev_flags.testFlag(SubTaskPrivate::WRITES_NEXT_START) && !next_flags.testFlag(SubTaskPrivate::READS_START)) &&
	       (next_flags.testFlag(SubTaskPrivate::WRITES_PREV_END) && !prev_flags.testFlag(SubTaskPrivate::READS_END));
}

inline bool SerialContainerPrivate::canInsert(const SubTask &stage, ContainerBasePrivate::const_iterator before) const {
	if (!ContainerBasePrivate::canInsert(stage))
		return false;

	// check connectedness
	bool at_end = (before == children().end());
	const SubTaskPrivate* next = (at_end) ? this : (*before)->pimpl_func();
	InterfaceFlags cur_flags = stage.pimpl_func()->announcedFlags();
	InterfaceFlags next_flags = next->deducedFlags();
	InterfaceFlags prev_flags = prev(before)->deducedFlags();

	// Do a simple check here only. A full connectivity check requires the full pipeline to be setup
	// Thus, here we reject when trying to connect to writers with each other
	if (bothWrite(prev_flags, cur_flags) || bothWrite(cur_flags, next_flags))
		return false;

	return true;
}

ContainerBasePrivate::iterator SerialContainerPrivate::insert(value_type &&stage, const_iterator before)
{
	assert(canInsert(*stage, before));
	bool at_begin = (before == children().begin());
	bool at_end = (before == children().end());

	SubTaskPrivate *cur = stage->pimpl_func();
	/* set pointer cache (prev_ends_ and next_starts_) of prev, current, and next stage */
	if (children().empty()) { // first child inserted
		setPrevEnds(cur, this->starts_);
		setNextStarts(cur, this->ends_);
	} else if (at_begin) {
		SubTaskPrivate *next = (*before)->pimpl_func();
		setPrevEnds(cur, this->starts_);
		setNextStarts(cur, next->starts_);
		setPrevEnds(next, cur->ends_);
	} else if (at_end) {
		const SubTaskPrivate *prev = this->prev(before);
		setNextStarts(prev, cur->starts_);
		setPrevEnds(cur, prev->ends_);
		setNextStarts(cur, this->ends_);
	} else {
		const SubTaskPrivate *prev = this->prev(before);
		SubTaskPrivate *next = (*before)->pimpl_func();
		setNextStarts(prev, cur->starts_);
		setPrevEnds(cur, prev->ends_);
		setNextStarts(cur, next->starts_);
		setPrevEnds(next, cur->ends_);
	}

	iterator it = ContainerBasePrivate::insert(std::move(stage), before);
}

inline const SubTaskPrivate* SerialContainerPrivate::prev(const_iterator it) const
{
#ifndef NDEBUG
	if (it != children().end()) {
		SubTaskPrivate* child = (*it)->pimpl_func();
		assert(parent(child) == this);
		assert(this->it(child) == it);
	}
#endif
	if (it == children().begin()) return this;
	return (*--it)->pimpl_func();
}

inline const SubTaskPrivate* SerialContainerPrivate::next(const_iterator it) const
{
#ifndef NDEBUG
	assert(it != children().end());
	SubTaskPrivate* child = (*it)->pimpl_func();
	assert(parent(child) == this);
	assert(this->it(child) == it);
#endif
	if (it == --children().end()) return this;
	return (*++it)->pimpl_func();
}

const SubTaskPrivate *SerialContainerPrivate::prev(const SubTaskPrivate *child) const
{
	return prev(it(child));
}

const SubTaskPrivate *SerialContainerPrivate::next(const SubTaskPrivate *child) const
{
	return next(it(child));
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
	return impl->canInsert(*subtask, impl->position(before));
}

bool SerialContainer::insert(value_type&& subtask, int before)
{
	IMPL(SerialContainer);

	ContainerBasePrivate::const_iterator where = impl->position(before);
	if (!impl->canInsert(*subtask, where))
		return false;

	impl->insert(std::move(subtask), where);
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
		if(!stage->pimpl_func()->canCompute())
			continue;
		std::cout << "Computing subtask '" << stage->getName() << "':" << std::endl;
		bool success = stage->pimpl_func()->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
	}
	return computed;
}

} }
