#include "stage_p.h"
#include <moveit_task_constructor/storage.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

namespace moveit { namespace task_constructor {

InterfaceState::InterfaceState(const planning_scene::PlanningSceneConstPtr &ps)
   : scene_(ps)
{
	registerID();
}

InterfaceState::InterfaceState(const InterfaceState &existing)
   : scene_(existing.scene())
{
	registerID();
}

InterfaceState::~InterfaceState()
{
	Repository<InterfaceState>::instance().remove(this);
	id_ = 0;
}

void InterfaceState::registerID()
{
	id_ = Repository<InterfaceState>::instance().add(this);
}


Interface::Interface(const Interface::NotifyFunction &notify)
   : notify_(notify)
{}

Interface::iterator Interface::add(InterfaceState &&state, SolutionBase* incoming, SolutionBase* outgoing) {
	if (!state.incomingTrajectories().empty() || !state.outgoingTrajectories().empty())
		throw std::runtime_error("expecting empty incoming/outgoing trajectories");
	if (!state.scene())
		throw std::runtime_error("expecting valid planning scene");

	assert(bool(incoming) ^ bool(outgoing)); // either incoming or outgoing is set
	emplace_back(state);
	iterator back = --end();
	// adjust subtrajectories ...
	if (incoming) incoming->setEndState(*back);
	if (outgoing) outgoing->setStartState(*back);
	// ... before calling notify callback
	if (notify_) notify_(back);
	return back;
}

Interface::iterator Interface::clone(const InterfaceState &state)
{
	emplace_back(InterfaceState(state));
	iterator back = --end();
	if (notify_) notify_(back);
	return back;
}


void SolutionBase::setCost(double cost) {
	cost_ = cost;
}


void SubTrajectory::fillMessage(moveit_task_constructor::Solution &msg) const {
	msg.sub_trajectory.emplace_back();
	moveit_task_constructor::SubTrajectory& t = msg.sub_trajectory.back();
	t.id = this->id();
	t.cost = this->cost();
	t.name = this->name();
	if (trajectory())
		trajectory()->getRobotTrajectoryMsg(t.trajectory);
	t.markers = this->markers();
}


} }
