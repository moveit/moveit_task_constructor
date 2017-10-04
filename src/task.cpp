#include "container_p.h"

#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/container.h>
#include <moveit_task_constructor/debug.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace moveit { namespace task_constructor {

class TaskPrivate : public SerialContainerPrivate {
	friend class Task;
	robot_model_loader::RobotModelLoaderPtr rml_;

public:
	TaskPrivate(Task* me, const std::string &name)
	   : SerialContainerPrivate(me, name)
	{
		initModel();
		initScene();
		initPlanner();
	}

	void initModel () {
		rml_.reset(new robot_model_loader::RobotModelLoader);
		if( !rml_->getModel() )
			throw Exception("Task failed to construct RobotModel");
	}
	void initScene() {
		assert(rml_);

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

		scene_ = std::make_shared<planning_scene::PlanningScene>(rml_->getModel());
		std::const_pointer_cast<planning_scene::PlanningScene>(scene_)->setPlanningSceneMsg(res.scene);
	}
	void initPlanner() {
		assert(rml_);
		assert(scene_);
		planner_ = std::make_shared<planning_pipeline::PlanningPipeline>(rml_->getModel(), ros::NodeHandle("move_group"));
	}
};


Task::Task(const std::string &name)
   : SerialContainer(new TaskPrivate(this, name))
{
}

void Task::add(pointer &&stage) {
	if (!stage)
		throw std::runtime_error("stage insertion failed: invalid stage pointer");
	if (!insert(std::move(stage)))
		throw std::runtime_error(std::string("insertion failed for stage: ") + stage->getName());
}

bool Task::plan(){
	NewSolutionPublisher debug(*this);

	while(ros::ok() && canCompute()) {
		if (compute()) {
			debug.publish();
			printState();
		} else
			break;
	}
	return false;
}

const robot_state::RobotState& Task::getCurrentRobotState() const {
	return pimpl_func()->scene_->getCurrentState();
}

void Task::printState(){
	ContainerBase::StageCallback processor = [](const SubTask& stage, int depth) -> bool {
		std::cout << std::string(2*depth, ' ') << stage << std::endl;
	};
	traverseStages(processor);
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
		for( SubTrajectory* successor : start.end->outgoingTrajectories() ){
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
	IMPL(Task);
	const TaskPrivate::array_type& children = impl->children();
	const size_t nr_of_trajectories = children.size();
	std::vector<SubTrajectory*> trace;
	trace.reserve(nr_of_trajectories);
	for(SubTrajectory& st : children.front()->pimpl_func()->trajectories())
		if( !traverseFullTrajectories(st, nr_of_trajectories, processor, trace) )
			return false;
	return true;
}

bool Task::processSolutions(const Task::SolutionCallback& processor) const {
	return const_cast<Task*>(this)->processSolutions(processor);
}

} }
