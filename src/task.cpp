#include "container_p.h"

#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/container.h>
#include <moveit_task_constructor/debug.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace moveit { namespace task_constructor {

Task::Task(ContainerBase::pointer &&container)
   : WrapperBase(std::string())
{
	task_starts_.reset(new Interface(Interface::NotifyFunction()));
	task_ends_.reset(new Interface(Interface::NotifyFunction()));

	insert(std::move(container));
	initModel();
}

void Task::initModel () {
	rml_.reset(new robot_model_loader::RobotModelLoader);
	if( !rml_->getModel() )
		throw Exception("Task failed to construct RobotModel");
}
void Task::initScene() {
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

planning_pipeline::PlanningPipelinePtr
Task::createPlanner(const robot_model::RobotModelConstPtr& model, const std::string& ns,
                    const std::string& planning_plugin_param_name,
                    const std::string& adapter_plugins_param_name) {
	typedef std::tuple<std::string, std::string, std::string, std::string> PlannerID;
	static std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline> > planner_cache;

	PlannerID id (model->getName(), ns, planning_plugin_param_name, adapter_plugins_param_name);
	auto it = planner_cache.find(id);

	planning_pipeline::PlanningPipelinePtr planner;
	if (it != planner_cache.cend())
		planner = it->second.lock();
	if (!planner) {
		planner = std::make_shared<planning_pipeline::PlanningPipeline>
		          (model, ros::NodeHandle(ns), planning_plugin_param_name, adapter_plugins_param_name);
		planner_cache[id] = planner;
	}
	return planner;
}

void Task::add(pointer &&stage) {
	if (!stage)
		throw std::runtime_error("stage insertion failed: invalid stage pointer");

	if (!wrapped()->insert(std::move(stage)))
		throw std::runtime_error(std::string("insertion failed for stage: ") + stage->name());
}

void Task::clear()
{
	wrapped()->clear();
}

Task::SolutionCallbackList::const_iterator Task::add(SolutionCallback &&cb)
{
	callbacks_.emplace_back(std::move(cb));
	return --callbacks_.cend();
}

void Task::erase(SolutionCallbackList::const_iterator which)
{
	callbacks_.erase(which);
}

void Task::reset()
{
	task_starts_->clear();
	task_ends_->clear();
	WrapperBase::reset();

	// connect my prevEnds() / nextStarts as WrapperBase will refer to them
	auto impl = pimpl();
	impl->setPrevEnds(task_ends_);
	impl->setNextStarts(task_starts_);
}

bool Task::canCompute() const
{
	return wrapped()->canCompute();
}

bool Task::compute()
{
	return wrapped()->compute();
}

bool Task::plan(){
	add(NewSolutionPublisher());

	reset();
	initScene();
	init(scene_);

	while(ros::ok() && canCompute()) {
	if (compute())
			printState();
		else
			break;
	}
	return numSolutions() > 0;
}

void Task::printState(){
	ContainerBase::StageCallback processor = [](const Stage& stage, int depth) -> bool {
		std::cout << std::string(2*depth, ' ') << stage << std::endl;
		return true;
	};
	wrapped()->traverseRecursively(processor);
}

size_t Task::numSolutions() const
{
	auto w = wrapped();
	// during initial insert() we call numSolutions(), but wrapped() is not yet defined
	return w ? w->numSolutions() : 0;
}

void Task::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	wrapped()->processSolutions(processor);
}

void Task::processSolutions(const Task::SolutionProcessor& processor) const {
	SolutionTrajectory solution;
	processSolutions([&solution, &processor](const SolutionBase& s) {
		solution.clear();
		s.flattenTo(solution);
		return processor(solution, s.cost());
	});
}

void Task::onNewSolution(SolutionBase &s)
{
	for (const auto& cb : callbacks_)
		cb(s);
}

inline ContainerBase* Task::wrapped()
{
	return static_cast<ContainerBase*>(WrapperBase::wrapped());
}

inline const ContainerBase* Task::wrapped() const
{
	return const_cast<Task*>(this)->wrapped();
}

} }
