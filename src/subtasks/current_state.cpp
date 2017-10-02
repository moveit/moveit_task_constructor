#include <moveit_task_constructor/subtasks/current_state.h>

namespace moveit { namespace task_constructor { namespace subtasks {

CurrentState::CurrentState(std::string name)
: Generator(name)
{
	ran_= false;
}

bool CurrentState::canCompute() const{
	return !ran_;
}

bool CurrentState::compute(){
	ran_= true;
	spawn(scene());

	return true;
}

} } }
