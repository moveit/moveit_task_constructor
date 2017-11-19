#include <moveit/visualization_tools/display_solution.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>

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
	// start scene is parent of end scene
	return scene_[idx_pair.first]->getParent();
}

const std::string &DisplaySolution::name(const IndexPair &idx_pair) const
{
	return name_[idx_pair.first];
}

void DisplaySolution::setFromMessage(const planning_scene::PlanningScenePtr& start_scene,
                                     const moveit_task_constructor_msgs::Solution &msg)
{
	if (msg.start_scene.robot_model_name != start_scene->getRobotModel()->getName()) {
		ROS_ERROR("Solution for model '%s' but model '%s' was expected",
		         msg.start_scene.robot_model_name .c_str(),
		         start_scene->getRobotModel()->getName().c_str());
		return;
	}

	// initialize parent scene from solution's start scene
	start_scene->setPlanningSceneMsg(msg.start_scene);
	start_scene_ = start_scene;
	planning_scene::PlanningScenePtr ref_scene = start_scene_->diff();

	scene_.resize(msg.sub_trajectory.size());
	trajectory_.resize(msg.sub_trajectory.size());
	name_.resize(msg.sub_trajectory.size());

	steps_ = 0;
	size_t i = 0;
	for (const auto& sub : msg.sub_trajectory) {
		trajectory_[i].reset(new robot_trajectory::RobotTrajectory(ref_scene->getRobotModel(), ""));
		trajectory_[i]->setRobotTrajectoryMsg(ref_scene->getCurrentState(), sub.trajectory);
		name_[i] = sub.name;
		steps_ += trajectory_[i]->getWayPointCount();

		ref_scene->setPlanningSceneDiffMsg(sub.scene_diff);
		scene_[i] = ref_scene;

		// create new reference scene for next iteration
		ref_scene = ref_scene->diff();
		++i;
	}
}

} // namespace moveit_rviz_plugin
