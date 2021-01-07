/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke, Artur Karoly */

#pragma once

#include <pluginlib/class_loader.h>
#include <moveit/macros/class_forward.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/grasp_provider.h>
#include <moveit/task_constructor/stages/place_provider.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace moveit {
namespace task_constructor {

namespace solvers {
MOVEIT_CLASS_FORWARD(CartesianPath)
MOVEIT_CLASS_FORWARD(PipelinePlanner)
}

namespace stages {

/** PickPlaceBase wraps the pipeline to pick or place an object with a given end effector.
 *
 * Picking consist of the following sub stages:
 * - linearly approaching the object along an approach direction/twist
 * - GraspProviderPlugin stage wrapped in an IK computation
 * - close hand
 * - attach object
 * - lift along along a given direction/twist
 *
 * Placing consist of the inverse order of stages:
 * - place down along a given direction
 * - PlaceProviderPlugin stage wrapped in an IK computation
 * - open hand
 * - detach the object
 * - linearly retract end effector
 */
class PickPlaceBase : public SerialContainer
{

	bool is_pick_;

	solvers::CartesianPathPtr cartesian_solver_;
	solvers::PipelinePlannerPtr sampling_planner_;

	Stage* move_there_stage_ = nullptr;
	Stage* compute_ik_stage_ = nullptr;
	moveit::task_constructor::stages::ModifyPlanningScene* set_collision_object_hand_stage_ = nullptr;
	moveit::task_constructor::stages::ModifyPlanningScene* allow_collision_object_support_stage_ = nullptr;
	moveit::task_constructor::stages::ModifyPlanningScene* forbid_collision_object_support_stage_ = nullptr;
	Stage* move_back_stage_ = nullptr;

	std::string provider_stage_plugin_name_;

	pluginlib::ClassLoader<GraspProviderBase> grasp_provider_class_loader_;
	pluginlib::ClassLoader<PlaceProviderBase> place_provider_class_loader_;

protected:
	moveit::task_constructor::stages::GraspProviderBase* grasp_stage_ = nullptr;
	moveit::task_constructor::stages::PlaceProviderBase* place_stage_ = nullptr;
	moveit::task_constructor::stages::ModifyPlanningScene* attach_detach_stage_ = nullptr;

public:
	PickPlaceBase(const std::string& name, const std::string& provider_stage_plugin_name, bool is_pick);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	/// -------------------------
	/// setters of own properties

	/// eef
	void setEndEffector(const std::string& eef) { properties().set<std::string>("eef", eef); }
	void setEndEffectorOpenClose(const std::string& open_pose_name, const std::string& close_pose_name){
		properties().set<std::string>("eef_group_open_pose", open_pose_name);
		properties().set<std::string>("eef_group_close_pose", close_pose_name);
	}

	/// object
	void setObject(const std::string& object) {properties().set<std::string>("object", object);}

	/// support surfaces
	void setSupportSurfaces(const std::vector<std::string>& surfaces) {properties().set<std::vector<std::string>>("support_surfaces", surfaces);}

	/// IK frame
	void setIKFrame(const geometry_msgs::PoseStamped& pose) { setProperty("ik_frame", pose); }
	void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);
	template <typename T>
	void setIKFrame(const T& p, const std::string& link) {
		Eigen::Isometry3d pose;
		pose = p;
		setIKFrame(pose, link);
	}
	void setIKFrame(const std::string& link) { setIKFrame(Eigen::Isometry3d::Identity(), link); }

	/// -------------------------
	/// setters of substage properties

	/// approach / place
	void setApproachPlace(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance);
	void setApproachPlace(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance);
	void setApproachPlace(const std::map<std::string, double>& joints);

	/// IK computation
	void setMaxIKSolutions(const uint32_t& max_ik_solutions);
	void setMinIKSolutionDistance(const double& min_ik_solution_distance);
	void setIgnoreIKCollisions(const bool& ignore_ik_collisions);

	/// lift / retract
	void setLiftRetract(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance);
	void setLiftRetract(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance);
	void setLiftRetract(const std::map<std::string, double>& joints);

	/// -------------------------
	/// getters, for further configuration

	solvers::CartesianPathPtr cartesianSolver() { return cartesian_solver_; }
	solvers::PipelinePlannerPtr samplingPlanner() {return sampling_planner_;}
	
	// Use this to retrieve a pointer to the GraspProviderPlugin object, to set its custom properties
	moveit::task_constructor::Stage* GraspProviderPlugin() {return grasp_stage_;}
};

/// specialization of PickPlaceBase to realize picking
class Pick : public PickPlaceBase
{
public:
	Pick(const std::string& name = "pick", const std::string& provider_stage_plugin_name = "moveit_task_constructor/GraspProviderDefault")
	  : PickPlaceBase(name, provider_stage_plugin_name, true) {}

	void setMonitoredStage(Stage* monitored);

	void setApproachMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance) {
		setApproachPlace(motion, min_distance, max_distance);
	}

	void setApproachMotion(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance) {
		setApproachPlace(direction, min_distance, max_distance);
	}

	void setApproachMotion(const std::map<std::string, double>& joints) { setApproachPlace(joints); }

	void setLiftMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance) {
		setLiftRetract(motion, min_distance, max_distance);
	}

	void setLiftMotion(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance) {
		setLiftRetract(direction, min_distance, max_distance);
	}

	void setLiftMotion(const std::map<std::string, double>& joints) { setLiftRetract(joints); }

	moveit::task_constructor::Stage* attachStage() {return attach_detach_stage_;}
};

/// specialization of PickPlaceBase to realize placing
class Place : public PickPlaceBase
{
public:
	Place(const std::string& name = "place", const std::string& provider_stage_plugin_name = "moveit_task_constructor/PlaceProviderDefault")
	  : PickPlaceBase(name, provider_stage_plugin_name, false) {}

	void setMonitoredStage(Stage* monitored);

	void setPlacePose(const geometry_msgs::PoseStamped& pose);

	void setRetractMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance) {
		setLiftRetract(motion, min_distance, max_distance);
	}

	void setRetractMotion(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance) {
		setLiftRetract(direction, min_distance, max_distance);
	}

	void setRetractMotion(const std::map<std::string, double>& joints) { setLiftRetract(joints); }

	void setPlaceMotion(const geometry_msgs::TwistStamped& motion, double min_distance, double max_distance) {
		setApproachPlace(motion, min_distance, max_distance);
	}
	void setPlaceMotion(const geometry_msgs::Vector3Stamped& direction, double min_distance, double max_distance) {
		setApproachPlace(direction, min_distance, max_distance);
	}

	void setPlaceMotion(const std::map<std::string, double>& joints) { setApproachPlace(joints); }

	moveit::task_constructor::Stage* detachStage() {return attach_detach_stage_;}
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
