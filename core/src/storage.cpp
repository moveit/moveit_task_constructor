#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor {

InterfaceState::InterfaceState(const planning_scene::PlanningSceneConstPtr &ps)
   : scene_(ps)
{
}

InterfaceState::InterfaceState(const InterfaceState &existing)
   : scene_(existing.scene())
{
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


void SubTrajectory::fillMessage(moveit_task_constructor_msgs::Solution &msg,
                                Introspection *introspection) const {
	msg.sub_trajectory.emplace_back();
	moveit_task_constructor_msgs::SubTrajectory& t = msg.sub_trajectory.back();
	t.id = introspection ? introspection->solutionId(*this) : 0;
	t.cost = this->cost();
	t.name = this->name();

	const Introspection *ci = introspection;
	t.stage_id = ci ? ci->stageId(this->creator()->me()) : 0;

	if (trajectory())
		trajectory()->getRobotTrajectoryMsg(t.trajectory);

	const auto& markers = this->markers();
	t.markers.clear();
	std::copy(markers.begin(), markers.end(), std::back_inserter(t.markers));

	this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);
}


} }
