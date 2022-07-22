/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2017, Hamburg University
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
/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/moveit_compat.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <chrono>
#include <functional>
#include <iterator>
#include <ros/console.h>

namespace moveit {
namespace task_constructor {
namespace stages {

ComputeIK::ComputeIK(const std::string& name, Stage::pointer&& child) : WrapperBase(name, std::move(child)) {
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("group", "name of active group (derived from eef if not provided)");
	p.declare<std::string>("default_pose", "", "default joint pose of active group (defines cost of IK)");
	p.declare<uint32_t>("max_ik_solutions", 1);
	p.declare<bool>("ignore_collisions", false);
	p.declare<double>("min_solution_distance", 0.1,
	                  "minimum distance between seperate IK solutions for the same target");

	// ik_frame and target_pose are read from the interface
	p.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose");
	p.declare<geometry_msgs::PoseStamped>("target_pose", "goal pose for ik frame");
}

void ComputeIK::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	pose_msg.pose = tf2::toMsg(pose);
	setIKFrame(pose_msg);
}

void ComputeIK::setTargetPose(const Eigen::Isometry3d& pose, const std::string& frame) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = frame;
	pose_msg.pose = tf2::toMsg(pose);
	setTargetPose(pose_msg);
}

// found IK solutions

struct IKSolution
{
	std::vector<double> joint_positions;
	bool feasible;
	collision_detection::Contact contact;
};

using IKSolutions = std::vector<IKSolution>;

namespace {

// ??? TODO: provide callback methods in PlanningScene class / probably not very useful here though...
// TODO: move into MoveIt core, lift active_components_only_ from fcl to common interface
bool isTargetPoseCollidingInEEF(const planning_scene::PlanningSceneConstPtr& scene,
                                robot_state::RobotState& robot_state, Eigen::Isometry3d pose,
                                const robot_model::LinkModel* link,
                                collision_detection::CollisionResult* collision_result = nullptr) {
	// consider all rigidly connected parent links as well
	const robot_model::LinkModel* parent = robot_model::RobotModel::getRigidlyConnectedParentLinkModel(link);
	if (parent != link)  // transform pose into pose suitable to place parent
		pose = pose * robot_state.getGlobalLinkTransform(link).inverse() * robot_state.getGlobalLinkTransform(parent);

	// place links at given pose
	robot_state.updateStateWithLinkAt(parent, pose);
	robot_state.updateCollisionBodyTransforms();

	// disable collision checking for parent links (except links fixed to root)
	auto acm = scene->getAllowedCollisionMatrix();
	std::vector<const std::string*> pending_links;  // parent link names that might be rigidly connected to root
	while (parent) {
		pending_links.push_back(&parent->getName());
		link = parent;
		const robot_model::JointModel* joint = link->getParentJointModel();
		parent = joint->getParentLinkModel();

		if (joint->getType() != robot_model::JointModel::FIXED) {
			for (const std::string* name : pending_links)
				acm.setDefaultEntry(*name, true);
			pending_links.clear();
		}
	}

	// check collision with the world using the padded version
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult result;
	req.contacts = (collision_result != nullptr);
	collision_detection::CollisionResult& res = collision_result ? *collision_result : result;
	scene->checkCollision(req, res, robot_state, acm);
	return res.collision;
}

std::string listCollisionPairs(const collision_detection::CollisionResult::ContactMap& contacts,
                               const std::string& separator) {
	std::string result;
	for (const auto& contact : contacts) {
		if (!result.empty())
			result.append(separator);
		result.append(contact.first.first).append(" - ").append(contact.first.second);
	}
	return result;
}

bool validateEEF(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                 const moveit::core::JointModelGroup*& jmg, std::string* msg) {
	try {
		const std::string& eef = props.get<std::string>("eef");
		if (!robot_model->hasEndEffector(eef)) {
			if (msg)
				*msg = "Unknown end effector: " + eef;
			return false;
		} else
			jmg = robot_model->getEndEffector(eef);
	} catch (const Property::undefined&) {
	}
	return true;
}
bool validateGroup(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                   const moveit::core::JointModelGroup* eef_jmg, const moveit::core::JointModelGroup*& jmg,
                   std::string* msg) {
	try {
		const std::string& group = props.get<std::string>("group");
		if (!(jmg = robot_model->getJointModelGroup(group))) {
			if (msg)
				*msg = "Unknown group: " + group;
			return false;
		}
	} catch (const Property::undefined&) {
		if (eef_jmg) {
			// derive group from eef
			const auto& parent = eef_jmg->getEndEffectorParentGroup();
			jmg = robot_model->getJointModelGroup(parent.first);
		}
	}
	return true;
}

}  // anonymous namespace

void ComputeIK::reset() {
	upstream_solutions_.clear();
	WrapperBase::reset();
}

void ComputeIK::init(const moveit::core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		WrapperBase::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	// all properties can be derived from the interface state
	// however, if they are defined already now, we validate here
	const auto& props = properties();
	const moveit::core::JointModelGroup* eef_jmg = nullptr;
	const moveit::core::JointModelGroup* jmg = nullptr;
	std::string msg;

	if (!validateEEF(props, robot_model, eef_jmg, &msg))
		errors.push_back(*this, msg);
	if (!validateGroup(props, robot_model, eef_jmg, jmg, &msg))
		errors.push_back(*this, msg);

	if (errors)
		throw errors;
}

void ComputeIK::onNewSolution(const SolutionBase& s) {
	assert(s.start() && s.end());
	assert(s.start()->scene() == s.end()->scene());  // wrapped child should be a generator

	// It's safe to store a pointer to the solution, as the generating stage stores it
	upstream_solutions_.push(&s);
}

bool ComputeIK::canCompute() const {
	return !upstream_solutions_.empty() || WrapperBase::canCompute();
}

void ComputeIK::compute() {
	if (WrapperBase::canCompute())
		WrapperBase::compute();

	if (upstream_solutions_.empty())
		return;

	const SolutionBase& s = *upstream_solutions_.pop();

	// -1 TODO: this should not be necessary in my opinion: Why do you think so?
	// It is, because the properties on the interface might change from call to call...
	// enforced initialization from interface ensures that new target_pose is read
	properties().performInitFrom(INTERFACE, s.start()->properties());
	const auto& props = properties();

	const planning_scene::PlanningSceneConstPtr& scene{ s.start()->scene() };

	const bool ignore_collisions = props.get<bool>("ignore_collisions");
	const auto& robot_model = scene->getRobotModel();
	const moveit::core::JointModelGroup* eef_jmg = nullptr;
	const moveit::core::JointModelGroup* jmg = nullptr;
	std::string msg;

	if (!validateEEF(props, robot_model, eef_jmg, &msg)) {
		ROS_WARN_STREAM_NAMED("ComputeIK", msg);
		return;
	}
	if (!validateGroup(props, robot_model, eef_jmg, jmg, &msg)) {
		ROS_WARN_STREAM_NAMED("ComputeIK", msg);
		return;
	}
	if (!eef_jmg && !jmg) {
		ROS_WARN_STREAM_NAMED("ComputeIK", "Neither eef nor group are well defined");
		return;
	}
	properties().property("timeout").setDefaultValue(jmg->getDefaultIKTimeout());

	// extract target_pose
	geometry_msgs::PoseStamped target_pose_msg = props.get<geometry_msgs::PoseStamped>("target_pose");
	if (target_pose_msg.header.frame_id.empty())  // if not provided, assume planning frame
		target_pose_msg.header.frame_id = scene->getPlanningFrame();

	Eigen::Isometry3d target_pose;
	tf2::fromMsg(target_pose_msg.pose, target_pose);
	if (target_pose_msg.header.frame_id != scene->getPlanningFrame()) {
		if (!scene->knowsFrameTransform(target_pose_msg.header.frame_id)) {
			ROS_WARN_STREAM_NAMED("ComputeIK",
			                      "Unknown reference frame for target pose: " << target_pose_msg.header.frame_id);
			return;
		}
		// transform target_pose w.r.t. planning frame
		target_pose = scene->getFrameTransform(target_pose_msg.header.frame_id) * target_pose;
	}

	// determine IK link from ik_frame
	const robot_model::LinkModel* link = nullptr;
	geometry_msgs::PoseStamped ik_pose_msg;
	const boost::any& value = props.get("ik_frame");
	if (value.empty()) {  // property undefined
		//  determine IK link from eef/group
		if (!(link = eef_jmg ? robot_model->getLinkModel(eef_jmg->getEndEffectorParentGroup().second) :
		                       jmg->getOnlyOneEndEffectorTip())) {
			ROS_WARN_STREAM_NAMED("ComputeIK", "Failed to derive IK target link");
			return;
		}
		ik_pose_msg.header.frame_id = link->getName();
		ik_pose_msg.pose.orientation.w = 1.0;
	} else {
		ik_pose_msg = boost::any_cast<geometry_msgs::PoseStamped>(value);
		Eigen::Isometry3d ik_pose;
		tf2::fromMsg(ik_pose_msg.pose, ik_pose);

		if (!scene->getCurrentState().knowsFrameTransform(ik_pose_msg.header.frame_id)) {
			ROS_WARN_STREAM_NAMED("ComputeIK", "ik frame unknown in robot: '" << ik_pose_msg.header.frame_id << "'");
			return;
		}
		ik_pose = scene->getCurrentState().getFrameTransform(ik_pose_msg.header.frame_id) * ik_pose;

		link = utils::getRigidlyConnectedParentLinkModel(scene->getCurrentState(), ik_pose_msg.header.frame_id);

		// transform target pose such that ik frame will reach there if link does
		target_pose = target_pose * ik_pose.inverse() * scene->getCurrentState().getFrameTransform(link->getName());
	}

	// validate placed link for collisions
	collision_detection::CollisionResult collisions;
	robot_state::RobotState sandbox_state{ scene->getCurrentState() };
	bool colliding =
	    !ignore_collisions && isTargetPoseCollidingInEEF(scene, sandbox_state, target_pose, link, &collisions);

	// frames at target pose and ik frame
	std::deque<visualization_msgs::Marker> frame_markers;
	rviz_marker_tools::appendFrame(frame_markers, target_pose_msg, 0.1, "target frame");
	rviz_marker_tools::appendFrame(frame_markers, ik_pose_msg, 0.1, "ik frame");
	// end-effector markers
	std::deque<visualization_msgs::Marker> eef_markers;
	// visualize placed end-effector
	auto appender = [&eef_markers](visualization_msgs::Marker& marker, const std::string& /*name*/) {
		marker.ns = "ik target";
		marker.color.a *= 0.5;
		eef_markers.push_back(marker);
	};
	const auto& links_to_visualize = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link)
	                                     ->getParentJointModel()
	                                     ->getDescendantLinkModels();
	if (colliding) {
		SubTrajectory solution;
		std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));
		generateCollisionMarkers(sandbox_state, appender, links_to_visualize);
		std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));
		solution.markAsFailure();
		// TODO: visualize collisions
		solution.setComment(s.comment() + " eef in collision: " + listCollisionPairs(collisions.contacts, ", "));
		auto colliding_scene{ scene->diff() };
		colliding_scene->setCurrentState(sandbox_state);
		spawn(InterfaceState(colliding_scene), std::move(solution));
		return;
	} else
		generateVisualMarkers(sandbox_state, appender, links_to_visualize);

	// determine joint values of robot pose to compare IK solution with for costs
	std::vector<double> compare_pose;
	const std::string& compare_pose_name = props.get<std::string>("default_pose");
	if (!compare_pose_name.empty()) {
		robot_state::RobotState compare_state(robot_model);
		compare_state.setToDefaultValues(jmg, compare_pose_name);
		compare_state.copyJointGroupPositions(jmg, compare_pose);
	} else
		scene->getCurrentState().copyJointGroupPositions(jmg, compare_pose);

	double min_solution_distance = props.get<double>("min_solution_distance");

	IKSolutions ik_solutions;
	auto is_valid = [scene, ignore_collisions, min_solution_distance,
	                 &ik_solutions](robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
	                                const double* joint_positions) {
		for (const auto& sol : ik_solutions) {
			if (jmg->distance(joint_positions, sol.joint_positions.data()) < min_solution_distance)
				return false;  // too close to already found solution
		}
		state->setJointGroupPositions(jmg, joint_positions);
		ik_solutions.emplace_back();
		auto& solution{ ik_solutions.back() };
		state->copyJointGroupPositions(jmg, solution.joint_positions);
		collision_detection::CollisionRequest req;
		collision_detection::CollisionResult res;
		req.contacts = true;
		req.max_contacts = 1;
		scene->checkCollision(req, res, *state);
		solution.feasible = ignore_collisions || !res.collision;
		if (res.contacts.size() > 0) {
			solution.contact = res.contacts.begin()->second.front();
		}
		return solution.feasible;
	};

	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	bool tried_current_state_as_seed = false;

	double remaining_time = timeout();
	auto start_time = std::chrono::steady_clock::now();
	while (ik_solutions.size() < max_ik_solutions && remaining_time > 0) {
		if (tried_current_state_as_seed)
			sandbox_state.setToRandomPositions(jmg);
		tried_current_state_as_seed = true;

		size_t previous = ik_solutions.size();
		bool succeeded = sandbox_state.setFromIK(jmg, target_pose, link->getName(), remaining_time, is_valid);

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;

		// for all new solutions (successes and failures)
		for (size_t i = previous; i != ik_solutions.size(); ++i) {
			// create a new scene for each solution as they will have different robot states
			planning_scene::PlanningScenePtr solution_scene = scene->diff();
			SubTrajectory solution;
			solution.setComment(s.comment());
			std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));

			if (ik_solutions[i].feasible)
				// compute cost as distance to compare_pose
				solution.setCost(s.cost() + jmg->distance(ik_solutions[i].joint_positions.data(), compare_pose.data()));
			else {  // found an IK solution, but this was not valid
				std::stringstream ss;
				ss << "Collision between '" << ik_solutions[i].contact.body_name_1 << "' and '"
				   << ik_solutions[i].contact.body_name_2 << "'";
				solution.markAsFailure(ss.str());
			}
			// set scene's robot state
			robot_state::RobotState& solution_state = solution_scene->getCurrentStateNonConst();
			solution_state.setJointGroupPositions(jmg, ik_solutions[i].joint_positions.data());
			solution_state.update();

			InterfaceState state(solution_scene);
			forwardProperties(*s.start(), state);

			// ik target link placement
			std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));

			spawn(std::move(state), std::move(solution));
		}

		// TODO: magic constant should be a property instead ("current_seed_only", or equivalent)
		// Yeah, you are right, these are two different semantic concepts:
		// One could also have multiple IK solutions derived from the same seed
		if (!succeeded && max_ik_solutions == 1)
			break;  // first and only attempt failed
	}

	if (ik_solutions.empty()) {  // failed to find any solution
		planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();
		SubTrajectory solution;

		solution.markAsFailure();
		solution.setComment(s.comment() + " no IK found");
		std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));

		// ik target link placement
		std_msgs::ColorRGBA tint_color;
		tint_color.r = 1.0;
		tint_color.g = 0.0;
		tint_color.b = 0.0;
		tint_color.a = 0.5;
		for (auto& marker : eef_markers)
			marker.color = tint_color;
		std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));

		spawn(InterfaceState(scene), std::move(solution));
	}
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
