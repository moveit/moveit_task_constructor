#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

ModifyPlanningScene::ModifyPlanningScene(std::string name)
  : PropagatingEitherWay(name)
{}

void ModifyPlanningScene::attachObjects(const Names& objects, const std::string& attach_link, bool attach)
{
	auto it_inserted = attach_objects_.insert(std::make_pair(attach_link, std::make_pair(Names(), attach)));
	Names &o = it_inserted.first->second.first;
	o.insert(o.end(), objects.begin(), objects.end());
}

template <>
void ModifyPlanningScene::enableCollisions<ModifyPlanningScene::Names>
(const ModifyPlanningScene::Names& first, const ModifyPlanningScene::Names& second, bool enable_collision) {
	collision_matrix_edits_.push_back(CollisionMatrixPairs({first, second, enable_collision}));
}

bool ModifyPlanningScene::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr to = apply(from.scene(), false);
	sendForward(from, InterfaceState(to), robot_trajectory::RobotTrajectoryPtr());
	return true;
}

bool ModifyPlanningScene::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from = apply(to.scene(), true);
	sendBackward(InterfaceState(from), to, robot_trajectory::RobotTrajectoryPtr());
	return true;
}

void ModifyPlanningScene::attachObjects(planning_scene::PlanningScene &scene, const std::pair<std::string, std::pair<Names, bool> >& pair, bool invert)
{
	moveit_msgs::AttachedCollisionObject obj;
	obj.link_name = pair.first;
	bool attach = pair.second.second;
	if (invert) attach = !attach;
	obj.object.operation = attach ? (int8_t) moveit_msgs::CollisionObject::ADD
	                              : (int8_t) moveit_msgs::CollisionObject::REMOVE;
	for (const std::string& name : pair.second.first) {
		obj.object.id = name;
		scene.processAttachedCollisionObjectMsg(obj);
	}
}

void ModifyPlanningScene::setCollisions(planning_scene::PlanningScene &scene, const CollisionMatrixPairs& pairs, bool invert)
{
	collision_detection::AllowedCollisionMatrix& acm = scene.getAllowedCollisionMatrixNonConst();
	bool enable = invert ? !pairs.enable : pairs.enable;
	if (pairs.second.empty()) {
		for (const auto &name : pairs.first)
			acm.setEntry(name, enable);
	} else
		acm.setEntry(pairs.first, pairs.second, enable);
}

// invert indicates, whether to detach instead of attach (and vice versa)
// as well as to disable instead of enable collision (and vice versa)
planning_scene::PlanningScenePtr ModifyPlanningScene::apply(const planning_scene::PlanningSceneConstPtr &scene, bool invert)
{
	planning_scene::PlanningScenePtr result = scene->diff();
	// attach/detach objects
	for (const auto &pair : attach_objects_)
		attachObjects(*result, pair, invert);

	// enable/disable collisions
	for (const auto &pairs : collision_matrix_edits_)
		setCollisions(*result, pairs, invert);
	return result;
}

} } }
