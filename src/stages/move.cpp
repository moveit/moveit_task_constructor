#include <moveit_task_constructor/stages/move.h>
#include <moveit_task_constructor/storage.h>
#include <moveit_task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace stages {

Move::Move(std::string name)
   : Connecting(name),
     timeout_(5.0)
{}

bool Move::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	planner_ = Task::createPlanner(scene->getRobotModel());
}

void Move::setGroup(std::string group){
	group_= group;
	mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_);
}

void Move::setPlannerId(std::string planner){
	planner_id_= planner;
}

void Move::setTimeout(double timeout){
	timeout_= timeout;
}

bool Move::compute(const InterfaceState &from, const InterfaceState &to) {
	mgi_->setJointValueTarget(to.state->getCurrentState());
	if( !planner_id_.empty() )
		mgi_->setPlannerId(planner_id_);
	mgi_->setPlanningTime(timeout_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep();
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(from.state, req, res))
		return false;

	// finish stage
	connect(from, to, res.trajectory_);

	return true;
}

} } }
