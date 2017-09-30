#include "subtask_p.h"
#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/debug.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace moveit { namespace task_constructor {

Task::Task(){
	rml_.reset(new robot_model_loader::RobotModelLoader);
	if( !rml_->getModel() )
		throw Exception("Task failed to construct RobotModel");

	ros::NodeHandle h;

	ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	client.waitForExistence();

	moveit_msgs::GetPlanningScene::Request req;
	moveit_msgs::GetPlanningScene::Response res;

	req.components.components =
		moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
		| moveit_msgs::PlanningSceneComponents::ROBOT_STATE
		| moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
		| moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
		| moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
		| moveit_msgs::PlanningSceneComponents::OCTOMAP
		| moveit_msgs::PlanningSceneComponents::TRANSFORMS
		| moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
		| moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING
		| moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

	if(!client.call(req, res)){
		throw Exception("Task failed to acquire current PlanningScene");
	}

	scene_.reset(new planning_scene::PlanningScene(rml_->getModel()));
	scene_->setPlanningSceneMsg(res.scene);

	planner_.reset(new planning_pipeline::PlanningPipeline(rml_->getModel(), ros::NodeHandle("move_group")));
}

Task::~Task(){
	subtasks_.clear();
	scene_.reset();
	planner_.reset();
}

void Task::clear(){
	subtasks_.clear();
}

bool Task::plan(){
	NewSolutionPublisher debug(*this);

	bool computed= true;
	while(ros::ok() && computed){
		computed= false;
		for( SubTaskPtr& subtask : subtasks_ ){
			if( !subtask->canCompute() )
				continue;
			std::cout << "Computing subtask '" << subtask->getName() << "':" << std::endl;
			bool success= subtask->compute();
			computed= true;
			std::cout << (success ? "succeeded" : "failed") << std::endl;
		}
		if(computed){
			debug.publish();
			printState();
		}
	}
	return false;
}

void Task::add( SubTaskPtr subtask ){
	subtask->setPlanningScene( scene_ );
	subtask->setPlanningPipeline( planner_ );

	if( !subtasks_.empty() ){
		subtask->impl_->addPredecessor( subtasks_.back() );
		subtasks_.back()->impl_->addSuccessor( subtask );
	}

	subtasks_.push_back( subtask );
}

const robot_state::RobotState& Task::getCurrentRobotState() const {
	return scene_->getCurrentState();
}

void Task::printState(){
	for( auto& st : subtasks_ ){
		std::cout
			<< st->impl_->getBeginning().size() << " -> "
			<< st->impl_->getTrajectories().size()
			<< " <- " << st->impl_->getEnd().size()
			<< " / " << st->getName()
			<< std::endl;
	}
}

namespace {
bool traverseFullTrajectories(
	SubTrajectory& start,
	int nr_of_trajectories,
	const Task::SolutionCallback& cb,
	std::vector<SubTrajectory*>& trace)
{
	bool ret= true;

	trace.push_back(&start);

	if(nr_of_trajectories == 1){
		ret= cb(trace);
	}
	else if( start.end ){
		for( SubTrajectory* successor : start.end->nextTrajectories() ){
			if( !traverseFullTrajectories(*successor, nr_of_trajectories-1, cb, trace) ){
				ret= false;
				break;
			}
		}
	}

	trace.pop_back();

	return ret;
}
}

bool Task::processSolutions(const Task::SolutionCallback& processor) {
	const size_t nr_of_trajectories= subtasks_.size();
	std::vector<SubTrajectory*> trace;
	trace.reserve(nr_of_trajectories);
	for(SubTrajectory& st : subtasks_.front()->impl_->trajectories_)
		if( !traverseFullTrajectories(st, subtasks_.size(), processor, trace) )
			return false;
	return true;
}

bool Task::processSolutions(const Task::SolutionCallback& processor) const {
	return const_cast<Task*>(this)->processSolutions(processor);
}

} }
