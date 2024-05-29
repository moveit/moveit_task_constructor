#include "models.h"
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace moveit::core;

RobotModelPtr getModel() {
	// suppress RobotModel errors and warnings
	if (rcutils_logging_set_logger_level("moveit_robot_model.robot_model", RCUTILS_LOG_SEVERITY_FATAL) != RCUTILS_RET_OK)
		throw std::runtime_error("Failed to set logger level to RCUTILS_LOG_SEVERITY_ERROR");

	// create dummy robot model
	moveit::core::RobotModelBuilder builder("robot", "base");
	builder.addChain("base->link1->link2->tip", "continuous");
	builder.addGroupChain("base", "link2", "group");
	builder.addGroupChain("link2", "tip", "eef_group");
	builder.addEndEffector("eef", "link2", "group", "eef_group");
	return builder.build();
}
