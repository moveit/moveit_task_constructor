#include "container_p.h"

#include <ros/console.h>

#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/range/adaptor/reversed.hpp>

namespace moveit { namespace task_constructor {

ContainerBasePrivate::const_iterator ContainerBasePrivate::position(int index) const {
	const_iterator position = children_.begin();
	if (index > 0) {
		for (auto end = children_.end(); index > 0 && position != end; --index)
			++position;
	} else if (++index <= 0) {
		container_type::const_reverse_iterator from_end = children_.rbegin();
		for (auto end = children_.rend(); index < 0 && from_end != end; ++index)
			++from_end;
		position = from_end.base();
	}
	return position;
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor,
                                          unsigned int cur_depth, unsigned int max_depth) const {
	if (cur_depth >= max_depth)
		return true;

	for (auto &stage : children_) {
		if (!processor(*stage, cur_depth))
			continue;
		ContainerBasePrivate *container = dynamic_cast<ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, cur_depth+1, max_depth);
	}
	return true;
}

bool ContainerBasePrivate::canCompute() const
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->canCompute();
}

bool ContainerBasePrivate::compute()
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->compute();
}

ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : Stage(impl)
{
}
PIMPL_FUNCTIONS(ContainerBase)

size_t ContainerBase::numChildren() const
{
	return pimpl()->children().size();
}

bool ContainerBase::traverseChildren(const ContainerBase::StageCallback &processor) const
{
	return pimpl()->traverseStages(processor, 0, 1);
}
bool ContainerBase::traverseRecursively(const ContainerBase::StageCallback &processor) const
{
	if (!processor(*this, 0))
		return false;
	return pimpl()->traverseStages(processor, 1, UINT_MAX);
}

bool ContainerBase::insert(Stage::pointer &&stage, int before)
{
	StagePrivate *impl = stage->pimpl();
	if (impl->parent() != nullptr || numSolutions() != 0) {
		ROS_ERROR("cannot re-parent stage");
		return false;
	}

	ContainerBasePrivate::const_iterator where = pimpl()->position(before);
	ContainerBasePrivate::iterator it = pimpl()->children_.insert(where, std::move(stage));
	impl->setHierarchy(pimpl(), it);
	return true;
}

void ContainerBase::clear()
{
	pimpl()->children_.clear();
}


SerialContainerPrivate::SerialContainerPrivate(SerialContainer *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	// these lists don't need a notify function, connections are handled by onNewSolution()
	pending_backward_.reset(new Interface(Interface::NotifyFunction()));
	pending_forward_.reset(new Interface(Interface::NotifyFunction()));
}

InterfaceFlags SerialContainerPrivate::announcedFlags() const {
	InterfaceFlags f;
	if (children().empty()) return f;
	f |= children().front()->pimpl()->announcedFlags() & INPUT_IF_MASK;
	f |= children().back()->pimpl()->announcedFlags() & OUTPUT_IF_MASK;
	return f;
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


struct SolutionCollector {
	SolutionCollector(const Stage::pointer& stage) : stopping_stage(stage->pimpl()) {}

	bool operator()(const SolutionBase& current, const std::vector<const SolutionBase*>& trace, double cost) {
		if (current.creator() != stopping_stage)
			return true; // not yet traversed to stopping_stage

		solutions.emplace_back(std::make_pair(trace, cost));
		return false; // we are done
	}

	std::list<std::pair<std::vector<const SolutionBase*>, double>> solutions;
	const StagePrivate* const stopping_stage;
};

void SerialContainerPrivate::onNewSolution(SolutionBase &current)
{
	const StagePrivate *creator = current.creator();

	// s.creator() should be one of our children
	assert(std::find_if(children().begin(), children().end(),
	                    [creator](const Stage::pointer& stage) { return stage->pimpl() == creator; } )
	       != children().end());

	SerialContainer *me = static_cast<SerialContainer*>(me_);

	// TODO: can we get rid of this and use a temporary when calling traverse()?
	std::vector<const SolutionBase*> trace; trace.reserve(children().size());

	// find all incoming trajectories connected to s
	SolutionCollector incoming(children().front());
	me->traverse<BACKWARD>(current, std::ref(incoming), trace);
	if (incoming.solutions.empty())
		return; // no connection to front()


	// find all outgoing trajectories connected to s
	SolutionCollector outgoing(children().back());
	me->traverse<FORWARD>(current, std::ref(outgoing), trace);
	if (outgoing.solutions.empty())
		return; // no connection to back()

	std::cerr << "new solution for: " << name() << std::endl;

	// add solutions for all combinations of incoming + s + outgoing
	std::vector<const SolutionBase*> solution;
	solution.reserve(children().size());
	for (auto& in : incoming.solutions) {
		for (auto& out : outgoing.solutions) {
			assert(solution.empty());
			// insert incoming solutions in reverse order
			solution.insert(solution.end(), in.first.rbegin(), in.first.rend());
			// insert current solution
			solution.push_back(&current);
			// insert outgoing solutions in normal order
			solution.insert(solution.end(), out.first.begin(), out.first.end());

			// TODO: store/announce solutions sorted by cost
			storeNewSolution(std::move(solution), in.second + current.cost() + out.second);
		}
	}
}

void SerialContainerPrivate::storeNewSolution(std::vector<const SolutionBase*> &&s, double cost)
{
	assert(!s.empty());
	const InterfaceState *internal_from = s.front()->start();
	const InterfaceState *internal_to = s.back()->end();

	// create new solution directly in solutions_ and get a reference to it
	solutions_.emplace_back(SerialSolution(this, std::move(s), cost));
	SerialSolution& solution = solutions_.back();

	// add solution to existing or new start state
	auto it = internal_to_my_starts_.find(internal_from);
	if (it != internal_to_my_starts_.end()) {
		// connect solution to existing start state
		solution.setStartState(*it->second);
	} else {
		// spawn a new state in previous stage
		prevEnds()->add(InterfaceState(*internal_from), NULL, &solution);
	}

	// add solution to existing or new end state
	it = internal_to_my_ends_.find(internal_to);
	if (it != internal_to_my_ends_.end()) {
		// connect solution to existing start state
		solution.setEndState(*it->second);
	} else {
		// spawn a new state in next stage
		nextStarts()->add(InterfaceState(*internal_to), &solution, NULL);
	}

	// inform parent about new solution
	if (parent())
		parent()->onNewSolution(solutions_.back());
}


SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}
PIMPL_FUNCTIONS(SerialContainer)

void SerialContainerPrivate::connect(StagePrivate* prev, StagePrivate* next) {
	prev->setNextStarts(next->starts());
	next->setPrevEnds(prev->ends());
}

void SerialContainer::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;

	auto impl = pimpl();
	// clear queues
	impl->internal_to_my_starts_.clear();
	impl->internal_to_my_ends_.clear();
	impl->solutions_.clear();

	// recursively init all children
	for (auto& stage : impl->children()) {
		try {
			// should be alled by derived classes too, but you never know...
			stage->Stage::init(scene);
			stage->init(scene);
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	// we need to have some children to do the actual work
	if (impl->children().empty()) {
		errors.push_back(*this, "no children");
		throw errors;
	}

	// initialize starts_ and ends_ interfaces
	auto cur = impl->children().begin();
	StagePrivate* child_impl = **cur;
	if (child_impl->starts())
		impl->starts_.reset(new Interface([impl, child_impl](const Interface::iterator& internal){
			// new state in our starts_ interface is copied to first child, remembering the link
			auto it = child_impl->starts()->clone(*internal);
			impl->internal_to_my_starts_.insert(std::make_pair(&*it, &*internal));
		}));

	auto last = --impl->children().end();
	if ((*cur)->pimpl()->ends())
		impl->ends_.reset(new Interface([impl, child_impl](const Interface::iterator& internal){
			// new state in our ends_ interface is copied to last child, remembering the link
			auto it = child_impl->ends()->clone(*internal);
			impl->internal_to_my_ends_.insert(std::make_pair(&*it, &*internal));
		}));

	/*** connect children ***/
	// first stage sends backward to pending_backward_
	(*cur)->pimpl()->setPrevEnds(impl->pending_backward_.get());

	// last stage sends forward to pending_forward_
	(*last)->pimpl()->setNextStarts(impl->pending_forward_.get());

	auto prev = cur; ++cur; // prev points to 1st, cur points to 2nd stage
	if (prev != last) {// we have more than one children
		auto next = cur; ++next; // next points to 3rd stage (or end)
		for (; cur != last; ++prev, ++cur, ++next) {
			impl->connect(**prev, **cur);
			impl->connect(**cur, **next);
		}
		// finally connect last == cur and prev stage
		impl->connect(**prev, **cur);
	}

	// validate connectivity of chain
	for (const Stage::pointer& stage : impl->children()) {
		try {
			stage->pimpl()->validate();
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	if (errors)
		throw errors;
}

bool SerialContainer::canCompute() const
{
	return !pimpl()->children().empty();
}

bool SerialContainer::compute()
{
	bool computed = false;
	for(const auto& stage : pimpl()->children()) {
		if(!stage->pimpl()->canCompute())
			continue;
		std::cout << "Computing stage '" << stage->name() << "':" << std::endl;
		bool success = stage->pimpl()->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
	}
	return computed;
}

size_t SerialContainer::numSolutions() const
{
	return pimpl()->solutions_.size();
}

template <TraverseDirection dir>
bool SerialContainer::traverse(const SolutionBase &start, const SolutionCallback &cb,
                               std::vector<const SolutionBase *> &trace, double trace_cost)
{
	if (!cb(start, trace, trace_cost))
		// stopping criterium met: stop traversal along dir
		return true; // but continue traversal of further trajectories

	bool result = false; // if no trajectory traversed, return false
	for (SolutionBase* successor : trajectories<dir>(start)) {
		trace.push_back(successor);
		trace_cost += successor->cost();

		result = traverse<dir>(*successor, cb, trace, trace_cost);

		trace_cost -= successor->cost();
		trace.pop_back();

		if (!result) break;
	}
	return result;
}

void SerialSolution::appendTo(std::vector<const SubTrajectory *> &solution) const
{
	solution.reserve(solution.size() + subsolutions_.size());
	for (const SolutionBase* s : subsolutions_)
		s->creator()->append(*s, solution);
}

} }
