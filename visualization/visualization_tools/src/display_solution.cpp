#include <moveit/visualization_tools/display_solution.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>

namespace moveit_rviz_plugin {

std::pair<size_t, size_t> DisplaySolution::indexPair(size_t index) const
{
	size_t part = 0;
	for (const auto& d : data_) {
		if (index < d.trajectory_->getWayPointCount())
			break;
		index -= d.trajectory_->getWayPointCount();
		++part;
	}
	assert(part < data_.size());
	assert(index < data_[part].trajectory_->getWayPointCount());
	return std::make_pair(part, index);
}

DisplaySolution::DisplaySolution(const DisplaySolution &master, uint32_t sub)
   : start_scene_(sub == 0 ? master.start_scene_ : master.data_[sub-1].scene_)
   , data_( { master.data_[sub] } )
{
	steps_ = data_.front().trajectory_->getWayPointCount();
}

float DisplaySolution::getWayPointDurationFromPrevious(const IndexPair &idx_pair) const
{
	return data_[idx_pair.first].trajectory_->getWayPointDurationFromPrevious(idx_pair.second);
}

const robot_state::RobotStatePtr& DisplaySolution::getWayPointPtr(const IndexPair &idx_pair) const
{
	return data_[idx_pair.first].trajectory_->getWayPointPtr(idx_pair.second);
}

const planning_scene::PlanningSceneConstPtr &DisplaySolution::scene(const IndexPair &idx_pair) const
{
	// start scene is parent of end scene
	return data_[idx_pair.first].scene_->getParent();
}

const std::string &DisplaySolution::name(const IndexPair &idx_pair) const
{
	return data_[idx_pair.first].name_;
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

	data_.resize(msg.sub_trajectory.size());

	steps_ = 0;
	size_t i = 0;
	for (const auto& sub : msg.sub_trajectory) {
		data_[i].trajectory_.reset(new robot_trajectory::RobotTrajectory(ref_scene->getRobotModel(), ""));
		data_[i].trajectory_->setRobotTrajectoryMsg(ref_scene->getCurrentState(), sub.trajectory);
		data_[i].name_ = sub.name;
		steps_ += data_[i].trajectory_->getWayPointCount();

		ref_scene->setPlanningSceneDiffMsg(sub.scene_diff);
		data_[i].scene_ = ref_scene;

		// create new reference scene for next iteration
		ref_scene = ref_scene->diff();
		++i;
	}
}

} // namespace moveit_rviz_plugin
