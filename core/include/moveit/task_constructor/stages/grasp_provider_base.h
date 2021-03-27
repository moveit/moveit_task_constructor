//
// Created by jafar_abdi on 3/27/21.
//

#ifndef MOVEIT_TASK_CONSTRUCTOR_CORE_GRASP_PROVIDER_BASE_H
#define MOVEIT_TASK_CONSTRUCTOR_CORE_GRASP_PROVIDER_BASE_H

#include "memory"
#include "moveit/task_constructor/container.h"
#include "grasp_provider_base.h"

namespace moveit {
namespace task_constructor {
namespace stages {
class GraspProviderBase : public GeneratePose
{
public:
	GraspProviderBase(const std::string& name = "grasp provider");

	void init(const std::shared_ptr<const moveit::core::RobotModel>& robot_model) override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }

	void setPreGraspPose(const std::string& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setPreGraspPose(const ::moveit_msgs::RobotState_<std::allocator<void>>& pregrasp) {
		properties().set("pregrasp", pregrasp);
	}
	void setGraspPose(const std::string& grasp) { properties().set("grasp", grasp); }
	void setGraspPose(const ::moveit_msgs::RobotState_<std::allocator<void>>& grasp) {
		properties().set("grasp", grasp);
	}

protected:
	void onNewSolution(const SolutionBase& s) override;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
#include <moveit/task_constructor/stages/generate_pose.h>
#endif  // MOVEIT_TASK_CONSTRUCTOR_CORE_GRASP_PROVIDER_BASE_H
