#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/subtask.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

moveit::task_constructor::Task::Task(){
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
		throw Exception("Task failed to aquire current PlanningScene");
	}

	scene_.reset(new planning_scene::PlanningScene(rml_->getModel()));
	scene_->setPlanningSceneMsg(res.scene);

	planner_.reset(new planning_pipeline::PlanningPipeline(rml_->getModel(), ros::NodeHandle("move_group")));
}

moveit::task_constructor::Task::~Task(){
	subtasks_.clear();
	scene_.reset();
}

void moveit::task_constructor::Task::addStart( SubTaskPtr subtask ){
	subtasks_.clear();
	addSubTask( subtask );
}

void moveit::task_constructor::Task::addAfter( SubTaskPtr subtask ){
	subtask->addPredecessor( subtasks_.back() );
	subtasks_.back()->addSuccessor( subtask );
	addSubTask( subtask );
}

bool moveit::task_constructor::Task::plan(){
	bool computed= true;
	while(ros::ok() && computed){
		computed= false;
		for( SubTaskPtr& subtask : subtasks_ ){
			if( !subtask->canCompute() )
				continue;
			std::cout << "Computing subtask '" << subtask->getName() << "': " << std::endl;
			bool success= subtask->compute();
			computed= true;
			std::cout << (success ? "succeeded" : "failed") << std::endl;
		}
		if(computed){
			printState();
			ros::Duration(0.5).sleep();
		}
	}
	return false;
}

void moveit::task_constructor::Task::addSubTask( SubTaskPtr subtask ){
	subtask->setPlanningScene( scene_ );
	subtask->setPlanningPipeline( planner_ );
	subtasks_.push_back( subtask );
}

void moveit::task_constructor::Task::printState(){
	for( auto& st : subtasks_ ){
		std::cout
			<< st->getBegin().size() << " -> "
			<< st->getTrajectories().size()
			<< " <- " << st->getEnd().size()
			<< " / " << st->getName()
			<< std::endl;
	}
}
