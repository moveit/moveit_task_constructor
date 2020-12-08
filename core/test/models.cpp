#include "models.h"
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace moveit::core;

RobotModelPtr getModel() {
	// create dummy robot model
	moveit::core::RobotModelBuilder builder("robot", "base");
	builder.addChain("base->link1->link2->tip", "continuous");
	builder.addGroupChain("base", "link2", "group");
	builder.addGroupChain("link2", "tip", "eef_group");
	builder.addEndEffector("eef", "link2", "group", "eef_group");
	return builder.build();
}

moveit::core::RobotModelPtr loadModel() {
	robot_model_loader::RobotModelLoader loader;
	return loader.getModel();
}
