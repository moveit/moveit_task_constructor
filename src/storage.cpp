#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor {

Interface::Interface(const Interface::NotifyFunction &notify)
   : notify_(notify)
{}

Interface::iterator Interface::add(const planning_scene::PlanningSceneConstPtr& ps, SubTrajectory* incoming, SubTrajectory* outgoing) {
	assert(bool(ps));
	assert(bool(incoming) ^ bool(outgoing)); // either incoming or outgoing is set
	emplace_back(InterfaceState(ps));
	iterator back = --end();
	// adjust subtrajectories ...
	if (incoming) incoming->setEndState(*back);
	if (outgoing) outgoing->setStartState(*back);
	// ... before calling notify callback
	if (notify_) notify_(back);
	return back;
}

} }
