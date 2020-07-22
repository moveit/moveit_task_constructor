/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Boston Cleek
   Desc:   Grasp generator stage using deep learning based grasp synthesizers
*/

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/action_base.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>

// #include <memory>
#include <functional>
// #include <iostream>

namespace moveit {
namespace task_constructor {
namespace stages {

constexpr char LOGNAME[] = "deep_grasp_generator";

/**
 * @brief Generate grasp candidates using deep learning approaches
 * @param ActionSpec - action message (action message name + "ACTION")
 * @details Interfaces with a deep learning based grasp library using a action client
 */
template <class ActionSpec>
class DeepGraspPose : public GeneratePose, ActionBase<ActionSpec>
{
private:
	typedef ActionBase<ActionSpec> ActionBaseT;
	ACTION_DEFINITION(ActionSpec);

public:
	/**
	 * @brief Constructor
	 * @param action_name - action namespace
	 * @param stage_name - name of stage
	 * @param goal_timeout - goal to completed time out (0 is considered infinite timeout)
	 * @param server_timeout - connection to server time out (0 is considered infinite timeout)
	 * @details Initialize the client and connect to server
	 */
	DeepGraspPose(const std::string& action_name, const std::string& stage_name = "generate grasp pose",
	              double goal_timeout = 0.0, double server_timeout = 0.0);

	/**
	 * @brief Composes the action goal and sends to server
	 */
	void composeGoal();

	/**
	 * @brief Monitors status of action goal
	 * @return true if grasp candidates are received within (optional) timeout
	 * @details This is a blocking call. It will wait until either grasp candidates
	 *          are received or the timeout has been reached.
	 */
	bool monitorGoal();

	void activeCallback() override;
	void feedbackCallback(const FeedbackConstPtr& feedback) override;
	void doneCallback(const actionlib::SimpleClientGoalState& state, const ResultConstPtr& result) override;

	void init(const core::RobotModelConstPtr& robot_model) override;
	void compute() override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }

	void setPreGraspPose(const std::string& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setPreGraspPose(const moveit_msgs::RobotState& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setGraspPose(const std::string& grasp) { properties().set("grasp", grasp); }
	void setGraspPose(const moveit_msgs::RobotState& grasp) { properties().set("grasp", grasp); }

protected:
	void onNewSolution(const SolutionBase& s) override;

private:
	bool found_candidates_;
	std::vector<geometry_msgs::PoseStamped> grasp_candidates_;
	std::vector<double> costs_;
};

template <class ActionSpec>
DeepGraspPose<ActionSpec>::DeepGraspPose(const std::string& action_name, const std::string& stage_name,
                                         double goal_timeout, double server_timeout)
  : GeneratePose(stage_name), ActionBaseT(action_name, false, goal_timeout, server_timeout), found_candidates_(false) {
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("object");
	p.declare<boost::any>("pregrasp", "pregrasp posture");
	p.declare<boost::any>("grasp", "grasp posture");

	ROS_INFO_NAMED(LOGNAME, "Waiting for connection to grasp generation action server...");
	ActionBaseT::clientPtr_->waitForServer(ros::Duration(ActionBaseT::server_timeout_));
	ROS_INFO_NAMED(LOGNAME, "Connected to server");
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::composeGoal() {
	Goal goal;
	goal.action_name = ActionBaseT::action_name_;
	ActionBaseT::clientPtr_->sendGoal(
	    goal, std::bind(&DeepGraspPose<ActionSpec>::doneCallback, this, std::placeholders::_1, std::placeholders::_2),
	    std::bind(&DeepGraspPose<ActionSpec>::activeCallback, this),
	    std::bind(&DeepGraspPose<ActionSpec>::feedbackCallback, this, std::placeholders::_1));

	ROS_INFO_NAMED(LOGNAME, "Goal sent to server: %s", ActionBaseT::action_name_.c_str());
}

template <class ActionSpec>
bool DeepGraspPose<ActionSpec>::monitorGoal() {
	// monitor timeout
	const bool monitor_timeout = ActionBaseT::goal_timeout_ > std::numeric_limits<double>::epsilon() ? true : false;
	const double timeout_time = ros::Time::now().toSec() + ActionBaseT::goal_timeout_;

	while (ActionBaseT::nh_.ok()) {
		ros::spinOnce();

		// timeout reached
		if (ros::Time::now().toSec() > timeout_time && monitor_timeout) {
			ActionBaseT::clientPtr_->cancelGoal();
			ROS_ERROR_NAMED(LOGNAME, "Grasp pose generator time out reached");
			return false;
		} else if (found_candidates_) {
			// timeout not reached (or not active) and grasps are found
			// only way return true
			break;
		}
	}

	return true;
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::activeCallback() {
	ROS_INFO_NAMED(LOGNAME, "Generate grasp goal now active");
	found_candidates_ = false;
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::feedbackCallback(const FeedbackConstPtr& feedback) {
	// each candidate should have a cost
	if (feedback->grasp_candidates.size() != feedback->costs.size()) {
		ROS_ERROR_NAMED(LOGNAME, "Invalid input: each grasp candidate needs an associated cost");
	} else {
		ROS_INFO_NAMED(LOGNAME, "Grasp generated feedback received %lu candidates: ", feedback->grasp_candidates.size());

		grasp_candidates_.resize(feedback->grasp_candidates.size());
		costs_.resize(feedback->costs.size());

		grasp_candidates_ = feedback->grasp_candidates;
		costs_ = feedback->costs;

		found_candidates_ = true;
	}
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::doneCallback(const actionlib::SimpleClientGoalState& state,
                                             const ResultConstPtr& result) {
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO_NAMED(LOGNAME, "Found grasp candidates (result): %s", result->grasp_state.c_str());
	} else {
		ROS_ERROR_NAMED(LOGNAME, "No grasp candidates found (state): %s",
		                ActionBaseT::clientPtr_->getState().toString().c_str());
	}
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef)) {
		errors.push_back(*this, "unknown end effector: " + eef);
	} else {
		// check availability of eef pose
		const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
		const std::string& name = props.get<std::string>("pregrasp");
		std::map<std::string, double> m;
		if (!jmg->getVariableDefaultPositions(name, m)) {
			errors.push_back(*this, "unknown end effector pose: " + name);
		}
	}

	if (errors) {
		throw errors;
	}
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::compute() {
	if (upstream_solutions_.empty()) {
		return;
	}
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	// set end effector pose
	const auto& props = properties();
	const std::string& eef = props.get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));

	// compose/send goal
	composeGoal();

	// monitor feedback/results
	// blocking function untill timeout reached or results received
	if (monitorGoal()) {
		// ROS_WARN_NAMED(LOGNAME, "number %lu: ",grasp_candidates_.size());
		for (unsigned int i = 0; i < grasp_candidates_.size(); i++) {
			InterfaceState state(scene);
			state.properties().set("target_pose", grasp_candidates_.at(i));
			props.exposeTo(state.properties(), { "pregrasp", "grasp" });

			SubTrajectory trajectory;
			trajectory.setCost(costs_.at(i));
			trajectory.setComment(std::to_string(i));

			// add frame at target pose
			rviz_marker_tools::appendFrame(trajectory.markers(), grasp_candidates_.at(i), 0.1, "grasp frame");

			spawn(std::move(state), std::move(trajectory));
		}
	}
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	if (!scene->knowsFrameTransform(object)) {
		const std::string msg = "object '" + object + "' not in scene";
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else {
			ROS_WARN_STREAM_NAMED(LOGNAME, msg);
		}
		return;
	}

	upstream_solutions_.push(&s);
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
