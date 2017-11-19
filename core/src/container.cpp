#include <moveit/task_constructor/container_p.h>

#include <moveit/task_constructor/introspection.h>
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

void ContainerBasePrivate::copyState(InterfaceState &external_state,
                                     Stage &child, bool to_start) {
	if (to_start) {
		auto internal = child.pimpl()->starts()->clone(external_state);
		internal_to_my_starts_.insert(std::make_pair(&*internal, &external_state));
	} else {
		auto internal = child.pimpl()->ends()->clone(external_state);
		internal_to_my_ends_.insert(std::make_pair(&*internal, &external_state));
	}
}

ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : Stage(impl)
{
}

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
	impl->setHierarchy(this, it);
	return true;
}

bool ContainerBase::remove(int pos)
{
	ContainerBasePrivate::const_iterator it = pimpl()->position(pos);
	pimpl()->children_.erase(it);
	return true;
}

void ContainerBase::clear()
{
	pimpl()->children_.clear();
}

void ContainerBase::reset()
{
	auto impl = pimpl();

	// recursively reset children
	for (auto& child: impl->children())
		child->reset();

	// clear mapping
	impl->internal_to_my_starts_.clear();
	impl->internal_to_my_ends_.clear();

	Stage::reset();
}

void ContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto& children = pimpl()->children();

	Stage::init(scene);

	// we need to have some children to do the actual work
	if (children.empty()) {
		errors.push_back(*this, "no children");
		throw errors;
	}

	// recursively init all children
	for (auto& child : children) {
		try {
			child->init(scene);
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	// validate connectivity of children
	for (auto& child : children) {
		try {
			child->pimpl()->validate();
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	// validate connectivity of this
	try {
		pimpl()->validate();
	} catch (InitStageException &e) {
		errors.append(e);
	}

	if (errors)
		throw errors;
}


SerialContainerPrivate::SerialContainerPrivate(SerialContainer *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	// these lists don't need a notify function, connections are handled by onNewSolution()
	pending_backward_.reset(new Interface(Interface::NotifyFunction()));
	pending_forward_.reset(new Interface(Interface::NotifyFunction()));
}


struct SolutionCollector {
	SolutionCollector(const Stage::pointer& stage) : stopping_stage(stage->pimpl()) {}

	bool operator()(const SolutionBase& current, const SerialContainer::solution_container& trace, double cost) {
		if (current.creator() != stopping_stage)
			return true; // not yet traversed to stopping_stage

		solutions.emplace_back(std::make_pair(trace, cost));
		return false; // we are done
	}

	std::list<std::pair<SerialContainer::solution_container, double>> solutions;
	const StagePrivate* const stopping_stage;
};

void SerialContainer::onNewSolution(SolutionBase &current)
{
	const StagePrivate *creator = current.creator();
	auto& children = pimpl()->children();

	// s.creator() should be one of our children
	assert(std::find_if(children.begin(), children.end(),
	                    [creator](const Stage::pointer& stage) { return stage->pimpl() == creator; } )
	       != children.end());

	SerialContainer::solution_container trace; trace.reserve(children.size());

	// find all incoming trajectories connected to s
	SolutionCollector incoming(children.front());
	traverse<BACKWARD>(current, std::ref(incoming), trace);
	if (incoming.solutions.empty())
		return; // no connection to front()


	// find all outgoing trajectories connected to s
	SolutionCollector outgoing(children.back());
	traverse<FORWARD>(current, std::ref(outgoing), trace);
	if (outgoing.solutions.empty())
		return; // no connection to back()

	// add solutions for all combinations of incoming + s + outgoing
	SerialContainer::solution_container solution;
	solution.reserve(children.size());
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
			pimpl()->storeNewSolution(std::move(solution), in.second + current.cost() + out.second);
		}
	}
}

void SerialContainerPrivate::storeNewSolution(SerialContainer::solution_container &&s, double cost)
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

	// perform default stage action on new solution
	newSolution(solutions_.back());
}


SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}

void SerialContainer::reset()
{
	auto impl = pimpl();

	// clear queues
	impl->solutions_.clear();
	impl->pending_backward_->clear();
	impl->pending_forward_->clear();

	// recursively reset children
	ContainerBase::reset();
}

void SerialContainerPrivate::connect(StagePrivate* prev, StagePrivate* next) {
	prev->setNextStarts(next->starts());
	next->setPrevEnds(prev->ends());
}

void SerialContainer::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// if there are no children, there is nothing to connect
	if (!impl->children().empty()) {

	// initialize starts_ and ends_ interfaces
	auto cur = impl->children().begin();
	Stage* child = cur->get();
	if (child->pimpl()->starts())
		impl->starts_.reset(new Interface([impl, child](const Interface::iterator& external){
			// new external state in our starts_ interface is copied to first child
			impl->copyState(*external, *child, true);
		}));

	auto last = --impl->children().end();
	child = last->get();
	if (child->pimpl()->ends())
		impl->ends_.reset(new Interface([impl, child](const Interface::iterator& external){
			// new external state in our ends_ interface is copied to last child
			impl->copyState(*external, *child, false);
		}));

	/*** connect children ***/
	// first stage sends backward to pending_backward_
	(*cur)->pimpl()->setPrevEnds(impl->pending_backward_);

	// last stage sends forward to pending_forward_
	(*last)->pimpl()->setNextStarts(impl->pending_forward_);

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

	// recursively init + validate all children
	// this needs to be done *after* initializing the connections
	ContainerBase::init(scene);

	// after initializing children, they might have changed their mind about reading...
	if (!impl->children().front()->pimpl()->starts())
		impl->starts_.reset();
	if (!impl->children().back()->pimpl()->ends())
		impl->ends_.reset();

	} else {
		// no children -> no reading
		impl->starts_.reset();
		impl->ends_.reset();

		// validate connectivity of this (would have been done in ContainerBase::init)
		try {
			pimpl()->validate();
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

void SerialContainer::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	for(const SolutionBase& s : pimpl()->solutions())
		if (!processor(s))
			break;
}

template <TraverseDirection dir>
bool SerialContainer::traverse(const SolutionBase &start, const SolutionProcessor &cb,
                               solution_container &trace, double trace_cost)
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

void SerialSolution::fillMessage(moveit_task_constructor_msgs::Solution &msg,
                                 Introspection* introspection = nullptr) const
{
	moveit_task_constructor_msgs::SubSolution sub_msg;
	sub_msg.id = introspection ? introspection->solutionId(*this) : 0;
	sub_msg.cost = this->cost();

	const Introspection *ci = introspection;
	sub_msg.stage_id = ci ? ci->stageId(this->creator()->me()) : 0;

	sub_msg.sub_solution_id.reserve(subsolutions_.size());
	if (introspection) {
		for (const SolutionBase* s : subsolutions_)
			sub_msg.sub_solution_id.push_back(introspection->solutionId(*s));
		msg.sub_solution.push_back(sub_msg);
	}

	msg.sub_trajectory.reserve(msg.sub_trajectory.size() + subsolutions_.size());
	for (const SolutionBase* s : subsolutions_)
		s->fillMessage(msg, introspection);
}


ParallelContainerBasePrivate::ParallelContainerBasePrivate(ParallelContainerBase *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
	starts_.reset(new Interface([me](const Interface::iterator& external){
		me->onNewStartState(*external);
	}));
	ends_.reset(new Interface([me](const Interface::iterator& external){
		me->onNewEndState(*external);
	}));
}

void ParallelContainerBase::onNewSolution(SolutionBase &s)
{
	auto impl = pimpl();
	WrappedSolution wrapped(impl, &s);
	// TODO: correctly clone start/end states from s to wrapped

	// store solution in our own list
	impl->solutions_.emplace_back(std::move(wrapped));
	// perform default stage action on new solution
	impl->newSolution(impl->solutions_.back());
}


ParallelContainerBase::ParallelContainerBase(ParallelContainerBasePrivate *impl)
   : ContainerBase(impl)
{}
ParallelContainerBase::ParallelContainerBase(const std::string &name)
   : ParallelContainerBase(new ParallelContainerBasePrivate(this, name))
{}

void ParallelContainerBase::reset()
{
	// clear solutions
	pimpl()->solutions_.clear();

	// recursively reset children
	ContainerBase::reset();
}

void ParallelContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// connect children such that they directly send this' prevEnds() / nextStarts()
	for (const Stage::pointer& stage : impl->children()) {
		StagePrivate *child = stage->pimpl();
		child->setPrevEnds(impl->prevEnds());
		child->setNextStarts(impl->nextStarts());
	}

	// recursively init + validate all children
	// this needs to be done *after* initializing the connections
	ContainerBase::init(scene);

	if (errors)
		throw errors;
}


size_t ParallelContainerBase::numSolutions() const
{
	return pimpl()->solutions_.size();
}

void ParallelContainerBase::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	for(const SolutionBase& s : pimpl()->solutions())
		if (!processor(s))
			break;
}


WrapperBase::WrapperBase(const std::string &name, Stage::pointer &&child)
   : ParallelContainerBase(new ParallelContainerBasePrivate(this, name))
{
	auto impl = pimpl();
	if (child) insert(std::move(child));
	// as a generator-like stage, we don't accept inputs
	impl->starts().reset();
	impl->ends().reset();
}

bool WrapperBase::insert(Stage::pointer &&stage, int before)
{
	// restrict num of children to one
	if (numChildren() > 0)
		return false;
	return ParallelContainerBase::insert(std::move(stage), before);
}

void WrapperBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	if (numChildren() != 1)
		throw InitStageException(*this, "no wrapped child");

	// init + validate children
	ParallelContainerBase::init(scene);
}

Stage* WrapperBase::wrapped()
{
	return pimpl()->children().empty() ? nullptr : pimpl()->children().front().get();
}

} }
