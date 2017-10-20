#include "container_p.h"

#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/container.h>
#include <moveit_task_constructor/debug.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace moveit { namespace task_constructor {

/** A Task wraps a single container. This wrapped container spawns its solutions
 *  into the prevEnds(), nextStarts() interface, which are provided by the wrappers
 *  end_ and start_ and interfaces respectively. */
class TaskPrivate : public WrapperBasePrivate {
	friend class Task;

	robot_model_loader::RobotModelLoaderPtr rml_;
	planning_scene::PlanningSceneConstPtr scene_; // initial scene
	std::list<Task::SolutionCallback> callbacks_; // functions called for each new solution

public:
	TaskPrivate(Task* me, const std::string &name = std::string())
	   : WrapperBasePrivate(me, name)
	{
		// provide external interfaces to the wrapped container
		starts_.reset(new Interface(Interface::NotifyFunction()));
		ends_.reset(new Interface(Interface::NotifyFunction()));
		// suppress insertion of this stage into another container
		// thus, this->parent() == this indicates the root
		setHierarchy(this, iterator());
		initModel();
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
};


Task::Task(ContainerBase::pointer &&container)
   : WrapperBase(new TaskPrivate(this))
{
	insert(std::move(container));
}
PIMPL_FUNCTIONS(Task)
PIMPL_FUNCTIONS(ContainerBase)

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
	auto& callbacks = pimpl()->callbacks_;
	callbacks.emplace_back(std::move(cb));
	return --callbacks.cend();
}

void Task::erase(SolutionCallbackList::const_iterator which)
{
	auto impl = pimpl();
	pimpl()->callbacks_.erase(which);
}

bool Task::canCompute() const
{
	return wrapped()->canCompute();
}

bool Task::compute()
{
	return wrapped()->compute();
}

void Task::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	auto impl = pimpl();
	// connect child
	StagePrivate *child = *wrapped();
	child->setPrevEnds(impl->starts());
	child->setNextStarts(impl->ends());

	WrapperBase::init(scene);
}

bool Task::plan(){
	auto impl = pimpl();
	add(NewSolutionPublisher());

	reset();
	impl->initScene();
	init(impl->scene_);

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
	std::vector<const SubTrajectory*> solution;
	processSolutions([&solution, &processor](const SolutionBase& s) {
		solution.clear();
		Task::flatten(s, solution);
		return processor(solution, s.cost());
	});
}

void Task::append(const SolutionBase &s, std::vector<const SubTrajectory *> &solution) const {
	wrapped()->pimpl()->append(s, solution);
}

void Task::onNewSolution(SolutionBase &s)
{
	for (const auto& cb : pimpl()->callbacks_)
		cb(s);
}

std::vector<const SubTrajectory *>& Task::flatten(const SolutionBase &s, std::vector<const SubTrajectory *> &result)
{
	s.creator()->append(s, result);
	return result;
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
