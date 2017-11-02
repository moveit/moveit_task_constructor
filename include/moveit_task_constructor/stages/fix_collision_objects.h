#pragma once

#include <moveit_task_constructor/stage.h>
#include <geometry_msgs/Pose.h>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace stages {

class FixCollisionObjects : public PropagatingEitherWay {
public:
	FixCollisionObjects(std::string name);

	bool computeForward(const InterfaceState& from) override;
	bool computeBackward(const InterfaceState& to) override;

	void setDistanceThreshold(double penetration);

protected:
	// apply stored modifications to scene
	void moveCollisionObject(planning_scene::PlanningScene &scene, geometry_msgs::Pose pose, const std::string& object, const std::string& base_frame);
	void checkCollisions(planning_scene::PlanningScene &scene, const std::string& object);
	planning_scene::PlanningScenePtr apply(const planning_scene::PlanningSceneConstPtr &scene, bool invert) const;

private:
	double penetration_;

};

} } }
