#include <moveit_task_constructor/subtasks/move.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace subtasks {

Move::Move(std::string name)
   : SubTask(name),
     timeout_(5.0)
{}

void Move::setGroup(std::string group){
	group_= group;
	mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_);
}

void Move::setLink(std::string link){
	link_= link;
}

void Move::setPlannerId(std::string planner){
	planner_id_= planner;
}

void Move::setTimeout(double timeout){
	timeout_= timeout;
}

/* TODO: implement this in compute
void Move::setFrom(std::string named_target){
	from_named_target_= named_target;
}

void Move::setTo(std::string named_target){
	to_named_target_= named_target;
}
*/

bool Move::canCompute(){
	return hasStatePair();
}

bool Move::compute(){
	assert( scene_->getRobotModel() );

	std::pair<InterfaceState&, InterfaceState&> state_pair= fetchStatePair();

	mgi_->setJointValueTarget(state_pair.second.state->getCurrentState());
	if( !planner_id_.empty() )
		mgi_->setPlannerId(planner_id_);
	mgi_->setPlanningTime(timeout_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep();
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(state_pair.first.state, req, res))
		return false;

	// finish subtask
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(res.trajectory_);
	trajectory.hasBeginning(state_pair.first);
	trajectory.hasEnding(state_pair.second);

	return true;
}

} } }
