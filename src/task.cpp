#include "container_p.h"

#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/container.h>
#include <moveit_task_constructor/introspection.h>

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <functional>

namespace moveit { namespace task_constructor {

Task::Task(const std::string& id, ContainerBase::pointer &&container)
   : WrapperBase(std::string()), id_(id)
{
	task_starts_.reset(new Interface(Interface::NotifyFunction()));
	task_ends_.reset(new Interface(Interface::NotifyFunction()));

	insert(std::move(container));
	initModel();

	// monitor state on commandline
	addTaskCallback(&printState);
	// enable introspection by default
	enableIntrospection(true);
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

Task::~Task()
{
	reset();
}

void Task::add(pointer &&stage) {
	if (!stage)
		throw std::runtime_error("stage insertion failed: invalid stage pointer");

	if (!stages()->insert(std::move(stage)))
		throw std::runtime_error(std::string("insertion failed for stage: ") + stage->name());
}

void Task::clear()
{
	stages()->clear();
}

void Task::enableIntrospection(bool enable)
{
	if (enable && !introspection_)
		introspection_.reset(new Introspection(*this));
	else if (!enable && introspection_)
		introspection_.reset();
}

Introspection &Task::introspection()
{
	enableIntrospection(true);
	return *introspection_;
}

Task::TaskCallbackList::const_iterator Task::addTaskCallback(TaskCallback &&cb)
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
	// signal introspection, that this task was reset
	if (introspection_)
		introspection_->reset();

	task_starts_->clear();
	task_ends_->clear();
	WrapperBase::reset();

	// connect my prevEnds() / nextStarts as WrapperBase will refer to them
	auto impl = pimpl();
	impl->setPrevEnds(task_ends_);
	impl->setNextStarts(task_starts_);
}

void Task::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	WrapperBase::init(scene);
	if (introspection_)
		introspection_->publishTaskDescription();
}

bool Task::canCompute() const
{
	return stages()->canCompute();
}

bool Task::compute()
{
	return stages()->compute();
}

bool Task::plan()
{
	reset();
	initScene();
	init(scene_);

	while(ros::ok() && canCompute()) {
		if (compute()) {
			for (const auto& cb : task_cbs_)
				cb(*this);
			if (introspection_)
				introspection_->publishTaskState();
		} else
			break;
	}
	return numSolutions() > 0;
}

size_t Task::numSolutions() const
{
	auto w = stages();
	// during initial insert() we call numSolutions(), but wrapped() is not yet defined
	return w ? w->numSolutions() : 0;
}

void Task::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	stages()->processSolutions(processor);
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

void Task::publishAllSolutions(bool wait)
{
	enableIntrospection(true);
	introspection_->publishAllSolutions(wait);
}

void Task::onNewSolution(SolutionBase &s)
{
	pimpl()->newSolution(s);
	if (introspection_)
		introspection_->publishSolution(s);
}

inline ContainerBase* Task::stages()
{
	return static_cast<ContainerBase*>(WrapperBase::wrapped());
}

inline const ContainerBase* Task::stages() const
{
	return const_cast<Task*>(this)->stages();
}

std::string Task::id() const
{
	return id_;
}

void Task::printState(const Task &t){
	ContainerBase::StageCallback processor = [](const Stage& stage, int depth) -> bool {
		std::cout << std::string(2*depth, ' ') << stage << std::endl;
		return true;
	};
	t.stages()->traverseRecursively(processor);
}

void fillInterfaceList(moveit_task_constructor::StageStatistics::_received_starts_type &c,
                   const InterfaceConstPtr& interface) {
	c.clear();
	if (!interface) return;
	for (const InterfaceState& state : *interface)
		c.push_back(state.id());
}

void fillStageStatistics(const Stage& stage, moveit_task_constructor::StageStatistics& s)
{
	const StagePrivate *simpl = stage.pimpl();

	fillInterfaceList(s.received_starts, simpl->starts());
	fillInterfaceList(s.received_ends, simpl->ends());
	fillInterfaceList(s.generated_starts, simpl->nextStarts());
	fillInterfaceList(s.generated_ends, simpl->prevEnds());

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
}

moveit_task_constructor::TaskDescription& Task::fillTaskDescription(moveit_task_constructor::TaskDescription &msg) const
{
	std::map<const Stage*, moveit_task_constructor::StageStatistics::_id_type> stage_to_id_map;
	stage_to_id_map[this] = 0; // ID for root

	ContainerBase::StageCallback stageProcessor =
	      [&stage_to_id_map, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent
		moveit_task_constructor::StageDescription desc;
		moveit_task_constructor::StageStatistics stat;
		desc.id = stat.id = stage_to_id_map.size();
		stage_to_id_map[&stage] = stat.id;

		desc.name = stage.name();
		desc.flags = stage.pimpl()->interfaceFlags();
		// TODO fill stage properties

		auto it = stage_to_id_map.find(stage.pimpl()->parent());
		assert (it != stage_to_id_map.cend());
		desc.parent_id = stat.parent_id = it->second;

		fillStageStatistics(stage, stat);

		// finally store in msg
		msg.description.push_back(std::move(desc));
		msg.statistics.push_back(std::move(stat));
		return true;
	};

	msg.description.clear();
	msg.statistics.clear();
	stages()->traverseRecursively(stageProcessor);

	msg.id = id();
	return msg;
}

moveit_task_constructor::TaskStatistics& Task::fillTaskStatistics(moveit_task_constructor::TaskStatistics &msg) const
{
	std::map<const Stage*, moveit_task_constructor::StageStatistics::_id_type> stage_to_id_map;
	stage_to_id_map[this] = 0; // ID for root

	ContainerBase::StageCallback stageProcessor =
	      [&stage_to_id_map, &msg](const Stage& stage, int) -> bool {
		// this method is called for each child stage of a given parent

		moveit_task_constructor::StageStatistics stat; // create new Stage msg
		stat.id = stage_to_id_map.size();
		stage_to_id_map[&stage] = stat.id;

		auto it = stage_to_id_map.find(stage.pimpl()->parent());
		assert (it != stage_to_id_map.cend());
		stat.parent_id = it->second;

		fillStageStatistics(stage, stat);

		// finally store in msg.stages
		msg.stages.push_back(std::move(stat));
		return true;
	};

	msg.stages.clear();
	stages()->traverseRecursively(stageProcessor);

	msg.id = id();
	return msg;
}

} }
