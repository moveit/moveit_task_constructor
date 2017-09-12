#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/subtask.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

moveit::task_constructor::Task::Task(){
	rml_.reset(new robot_model_loader::RobotModelLoader);
	if( !rml_->getModel() )
		throw Exception("Task failed to construct RobotModel");

	ros::NodeHandle h;

	pub = h.advertise<moveit_msgs::DisplayTrajectory>("task_plan", 30);

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

moveit::task_constructor::Task::~Task(){
	subtasks_.clear();
	scene_.reset();
	planner_.reset();
}

void moveit::task_constructor::Task::clear(){
	subtasks_.clear();
}

bool moveit::task_constructor::Task::plan(){
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
			printState();
			publishPlans();
		}
	}
	return false;
}

void moveit::task_constructor::Task::add( SubTaskPtr subtask ){
	subtask->setPlanningScene( scene_ );
	subtask->setPlanningPipeline( planner_ );

	if( !subtasks_.empty() ){
		subtask->addPredecessor( subtasks_.back() );
		subtasks_.back()->addSuccessor( subtask );
	}

	subtasks_.push_back( subtask );
}

void moveit::task_constructor::Task::printState(){
	for( auto& st : subtasks_ ){
		std::cout
			<< st->getBeginning().size() << " -> "
			<< st->getTrajectories().size()
			<< " <- " << st->getEnd().size()
			<< " / " << st->getName()
			<< std::endl;
	}
}

namespace {
bool traverseFullTrajectories(
	moveit::task_constructor::SubTrajectory& start,
	int nr_of_trajectories,
	moveit::task_constructor::Task::SolutionCallback& cb,
	std::vector<moveit::task_constructor::SubTrajectory*>& trace)
{
	bool ret= true;

	trace.push_back(&start);

	if(nr_of_trajectories == 1){
		ret= cb(trace);
	}
	else if( start.end ){
		for( moveit::task_constructor::SubTrajectory* successor : start.end->next_trajectory ){
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

bool moveit::task_constructor::Task::processSolutions(moveit::task_constructor::Task::SolutionCallback& processor ){
	const size_t nr_of_trajectories= subtasks_.size();
	std::vector<moveit::task_constructor::SubTrajectory*> trace;
	trace.reserve(nr_of_trajectories);
	for(SubTrajectory& subtraj : subtasks_.front()->getTrajectories())
		if( !traverseFullTrajectories(subtraj, subtasks_.size(), processor, trace) )
			return false;
	return true;
}

namespace {
bool publishSolution(ros::Publisher& pub, moveit_msgs::DisplayTrajectory& dt, std::vector<moveit::task_constructor::SubTrajectory*>& solution){
		bool all_flagged= true;
		for(auto& t : solution){
			all_flagged= all_flagged && t->flag;
		}

		if( all_flagged ){
			return true;
		}

		for(auto& t : solution){
			if(t->trajectory){
				dt.trajectory.emplace_back();
				t->trajectory->getRobotTrajectoryMsg(dt.trajectory.back());
			}
			t->flag= true;
		}

		std::cout << "publishing solution" << std::endl;
		pub.publish(dt);

		return true;
}
}

void moveit::task_constructor::Task::publishPlans(){
	moveit_msgs::DisplayTrajectory dt;
	robot_state::robotStateToRobotStateMsg(scene_->getCurrentState(), dt.trajectory_start);
	dt.model_id= scene_->getRobotModel()->getName();

	moveit::task_constructor::Task::SolutionCallback processor= std::bind(
		&publishSolution, pub, dt, std::placeholders::_1);

	processSolutions(processor);

}
