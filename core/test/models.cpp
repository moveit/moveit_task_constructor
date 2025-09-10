#include "models.h"
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace moveit::core;

RobotModelPtr getModel() {
	// suppress RobotModel errors and warnings
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME ".moveit_core.robot_model", ros::console::levels::Fatal);

	// create dummy robot model
	moveit::core::RobotModelBuilder builder("robot", "base");
	builder.addChain("base->link1->link2->tip", "continuous");
	builder.addGroupChain("base", "link2", "group");
	builder.addGroupChain("link2", "tip", "eef_group");
	builder.addEndEffector("eef", "link2", "group", "eef_group");
	return builder.build();
}

moveit::core::RobotModelPtr loadModel() {
	static robot_model_loader::RobotModelLoader loader;
	return loader.getModel();
}
