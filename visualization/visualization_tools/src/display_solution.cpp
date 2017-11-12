#include <moveit/visualization_tools/display_solution.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_rviz_plugin {

std::pair<size_t, size_t> DisplaySolution::indexPair(size_t index) const
{
	size_t part = 0;
	for (const auto& t : trajectory_) {
		if (index < t->getWayPointCount())
			break;
		index -= t->getWayPointCount();
		++part;
	}
	assert(part < trajectory_.size());
	assert(index < trajectory_[part]->getWayPointCount());
	return std::make_pair(part, index);
}

float DisplaySolution::getWayPointDurationFromPrevious(const IndexPair &idx_pair) const
{
	return trajectory_[idx_pair.first]->getWayPointDurationFromPrevious(idx_pair.second);
}

const robot_state::RobotStatePtr& DisplaySolution::getWayPointPtr(const IndexPair &idx_pair) const
{
	return trajectory_[idx_pair.first]->getWayPointPtr(idx_pair.second);
}

const planning_scene::PlanningSceneConstPtr &DisplaySolution::scene(const IndexPair &idx_pair) const
{
	return scene_[idx_pair.first];
}

const std::string &DisplaySolution::name(const IndexPair &idx_pair) const
{
	return name_[idx_pair.first];
}

void DisplaySolution::setFromMessage(const planning_scene::PlanningSceneConstPtr& parent,
                                     const moveit_task_constructor_msgs::Solution &msg)
{
	planning_scene::PlanningScenePtr ref_scene = parent->diff();

	scene_.resize(msg.sub_trajectory.size());
	trajectory_.resize(msg.sub_trajectory.size());
	name_.resize(msg.sub_trajectory.size());

	steps_ = 0;
	size_t i = 0;
	for (const auto& sub : msg.sub_trajectory) {
		scene_[i] = ref_scene;
		trajectory_[i].reset(new robot_trajectory::RobotTrajectory(ref_scene->getRobotModel(), ""));
		trajectory_[i]->setRobotTrajectoryMsg(ref_scene->getCurrentState(), sub.trajectory);

		name_[i] = sub.name;
		steps_ += trajectory_[i]->getWayPointCount();

		// create new reference scene based of diffs
		ref_scene = ref_scene->diff();
		ref_scene->setPlanningSceneDiffMsg(sub.scene_diff);
		++i;
	}
}

} // namespace moveit_rviz_plugin
