//
// Created by llach on 24.11.17.
//
// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/container.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState)
MOVEIT_CLASS_FORWARD(JointModelGroup)
}
}

namespace moveit { namespace task_constructor { namespace stages {

/** Wrapper for any pose generator stage to compute IK poses for a Cartesian pose.
 *
 * The wrapper reads a target_pose from the interface state of solutions provided
 * by the wrapped stage. This Cartesian pose (PoseStamped msg) is used as a goal
 * pose for inverse kinematics.
 * Usually, the end effector's parent link or the group's tip link is used as
 * the IK frame, which should be moved to the goal frame. However, any other
 * IK frame can be defined (which is linked to the tip of the group).
 */
class ComputeIK : public WrapperBase {
public:
	ComputeIK(const std::string &name, pointer &&child = Stage::pointer());

	void init(const core::RobotModelConstPtr &robot_model);
	void onNewSolution(const SolutionBase &s) override;

	void setTimeout(double timeout);
	void setEndEffector(const std::string& eef);

	/// setters for IK frame
	void setIKFrame(const geometry_msgs::PoseStamped &pose);
	void setIKFrame(const Eigen::Affine3d& pose, const std::string& link);
	template <typename T>
	void setIKFrame(const T& p, const std::string& link) {
		Eigen::Affine3d pose; pose = p;
		setIKFrame(pose, link);
	}
	void setIKFrame(const std::string& link) {
		setIKFrame(Eigen::Affine3d::Identity(), link);
	}

	/// setters for target pose
	void setTargetPose(const geometry_msgs::PoseStamped &pose);
	void setTargetPose(const Eigen::Affine3d& pose, const std::string& frame = "");
	template <typename T>
	void setTargetPose(const T& p, const std::string& frame = "") {
		Eigen::Affine3d pose; pose = p;
		setTargetPose(pose, frame);
	}

	void setMaxIKSolutions(uint32_t n);
	void setIgnoreCollisions(bool flag);
};

} } }
