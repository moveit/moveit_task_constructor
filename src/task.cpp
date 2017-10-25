#include "container_p.h"

#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/container.h>
#include <moveit_task_constructor/debug.h>
#include <moveit_task_constructor/introspection_publisher.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <functional>

namespace moveit { namespace task_constructor {

static size_t g_task_id = 0;

Task::Task(ContainerBase::pointer &&container)
   : WrapperBase(std::string()), id_(++g_task_id)
{
	task_starts_.reset(new Interface(Interface::NotifyFunction()));
	task_ends_.reset(new Interface(Interface::NotifyFunction()));

	insert(std::move(container));
	initModel();

	// monitor state on commandline
	add(&printState);
	// publish state
	add(std::bind(&IntrospectionPublisher::publish,
	              IntrospectionPublisher::instance(),
	              std::placeholders::_1));
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
	id_ = ++g_task_id;
}

Task::SolutionCallbackList::const_iterator Task::add(SolutionCallback &&cb)
{
	solution_cbs_.emplace_back(std::move(cb));
	return --solution_cbs_.cend();
}

void Task::erase(SolutionCallbackList::const_iterator which)
{
	solution_cbs_.erase(which);
}

Task::TaskCallbackList::const_iterator Task::add(TaskCallback &&cb)
{
	task_cbs_.emplace_back(std::move(cb));
	return --task_cbs_.cend();
}

void Task::erase(TaskCallbackList::const_iterator which)
{
	task_cbs_.erase(which);
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
	reset();
	initScene();
	init(scene_);

	while(ros::ok() && canCompute()) {
		if (compute()) {
			for (const auto& cb : task_cbs_)
				cb(*this);
		} else
			break;
	}
	return numSolutions() > 0;
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
	::moveit_task_constructor::Solution msg;
	msg.task_id = id();
	processSolutions([&msg, &processor](const SolutionBase& s) {
		msg.sub_solution.clear();
		msg.sub_trajectory.clear();
		s.fillMessage(msg);
		return processor(msg, s.cost());
	});
}

void Task::onNewSolution(SolutionBase &s)
{
	for (const auto& cb : solution_cbs_)
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

std::string Task::id() const
{
	ros::NodeHandle n;
	return std::to_string(id_) + '@' + n.getNamespace();
}

void Task::printState(const Task &t){
	ContainerBase::StageCallback processor = [](const Stage& stage, int depth) -> bool {
		std::cout << std::string(2*depth, ' ') << stage << std::endl;
		return true;
	};
	t.wrapped()->traverseRecursively(processor);
}

void fillStateList(moveit_task_constructor::Stage::_received_starts_type &c,
                   const InterfaceConstPtr& interface) {
	c.clear();
	if (!interface) return;
	for (const InterfaceState& state : *interface)
		c.push_back(state.id());
}

moveit_task_constructor::Task& Task::fillMessage(moveit_task_constructor::Task &msg) const
{
	std::map<const Stage*, moveit_task_constructor::Stage::_id_type> stage_to_id_map;
	ContainerBase::StageCallback stageProcessor =
	      [&stage_to_id_map, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		const StagePrivate *simpl = stage.pimpl();

		moveit_task_constructor::Stage s; // create new Stage msg
		s.id = stage_to_id_map.size();
		stage_to_id_map[&stage] = s.id;
		s.name = stage.name();
		s.flags = stage.pimpl()->interfaceFlags();
		auto it = stage_to_id_map.find(simpl->parent());
		assert (it != stage_to_id_map.cend());
		s.parent_id = it->second;

		fillStateList(s.received_starts, simpl->starts());
		fillStateList(s.received_ends, simpl->ends());
		fillStateList(s.generated_starts, simpl->nextStarts());
		fillStateList(s.generated_ends, simpl->prevEnds());

		// insert solution IDs
		Stage::SolutionProcessor solutionProcessor = [&s](const SolutionBase& solution) {
			s.solved.push_back(solution.id());
			return true;
		};
		stage.processSolutions(solutionProcessor);

		solutionProcessor = [&s](const SolutionBase& solution) {
			s.failed.push_back(solution.id());
			return true;
		};
		stage.processFailures(solutionProcessor);

		// finally store in msg.stages
		msg.stages.push_back(std::move(s));
		return true;
	};

	msg.id = id();
	msg.stages.clear();
	stage_to_id_map[this] = 0; // ID for root
	wrapped()->traverseRecursively(stageProcessor);
	return msg;
}

} }
