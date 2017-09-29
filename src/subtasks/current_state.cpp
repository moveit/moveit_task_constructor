#include <moveit_task_constructor/subtasks/current_state.h>

namespace moveit { namespace task_constructor { namespace subtasks {

CurrentState::CurrentState(std::string name)
: Generator(name)
{
	ran_= false;
}

bool CurrentState::canCompute(){
	return !ran_;
}

bool CurrentState::compute(){
	ran_= true;
	assert( scene_ );
	spawn(scene_);

	return true;
}

} } }
