#include <moveit_task_constructor/stages/current_state.h>
#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor { namespace stages {

CurrentState::CurrentState(std::string name)
: Generator(name)
{
	ran_= false;
}

bool CurrentState::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	scene_ = scene;
	ran_= false;
}

bool CurrentState::canCompute() const{
	return !ran_;
}

bool CurrentState::compute(){
	ran_= true;
	spawn(InterfaceState(scene_));

	return true;
}

} } }
