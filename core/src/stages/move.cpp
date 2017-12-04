#include <moveit/task_constructor/stages/move.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace stages {

Move::Move(std::string name)
   : Connecting(name)
{
	auto& p = properties();
	p.declare<double>("timeout", 5.0, "planning timeout");
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("planner", "", "planner id");
}

void Move::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Connecting::init(scene);
	planner_ = Task::createPlanner(scene->getRobotModel());
}

void Move::setGroup(const std::string& group){
	setProperty("group", group);
}

void Move::setPlannerId(const std::string& planner){
	setProperty("planner", planner);
}

void Move::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

bool Move::compute(const InterfaceState &from, const InterfaceState &to) {
	const auto& props = properties();
	moveit::planning_interface::MoveGroupInterface mgi(props.get<std::string>("group"));
	mgi.setJointValueTarget(to.scene()->getCurrentState());

	const std::string planner_id = props.get<std::string>("planner");
	if( !planner_id.empty() )
		mgi.setPlannerId(planner_id);
	mgi.setPlanningTime(props.get<double>("timeout"));

	::planning_interface::MotionPlanRequest req;
	mgi.constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep();
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(from.scene(), req, res))
		return false;

	// finish stage
	connect(from, to, res.trajectory_);

	return true;
}

} } }
