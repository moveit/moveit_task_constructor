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

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <chrono>
#include <functional>
#include <iterator>
#include <ros/console.h>

namespace moveit { namespace task_constructor { namespace stages {

ComputeIK::ComputeIK(const std::string &name, Stage::pointer &&child)
   : WrapperBase(name, std::move(child))
{
	auto& p = properties();
	p.declare<double>("timeout", 1.0);
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("group", "name of active group (derived from eef if not provided)");
	p.declare<std::string>("default_pose", "", "default joint pose of active group (defines cost of IK)");
	p.declare<uint32_t>("max_ik_solutions", 1);
	p.declare<bool>("ignore_collisions", false);

	// ik_frame and target_pose are read from the interface
	p.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose");
	p.declare<geometry_msgs::PoseStamped>("target_pose", "goal pose for ik frame");
	p.configureInitFrom(Stage::INTERFACE, {"target_pose"});
}

void ComputeIK::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

void ComputeIK::setEndEffector(const std::string &eef){
	setProperty("eef", eef);
}

void ComputeIK::setIKFrame(const geometry_msgs::PoseStamped &pose)
{
	setProperty("ik_frame", pose);
}

void ComputeIK::setIKFrame(const Eigen::Affine3d &pose, const std::string &link)
{
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setIKFrame(pose_msg);
}

void ComputeIK::setTargetPose(const geometry_msgs::PoseStamped &pose)
{
	setProperty("target_pose", pose);
}

void ComputeIK::setTargetPose(const Eigen::Affine3d &pose, const std::string &frame)
{
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = frame;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setTargetPose(pose_msg);
}

void ComputeIK::setMaxIKSolutions(uint32_t n){
	setProperty("max_ik_solutions", n);
}

void ComputeIK::setIgnoreCollisions(bool flag)
{
	setProperty("ignore_collisions", flag);
}

// found IK solutions with a flag indicating validity
typedef std::vector<std::vector<double>> IKSolutions;

namespace {

bool isTargetPoseColliding(const planning_scene::PlanningScenePtr& scene,
                           Eigen::Affine3d pose, const robot_model::LinkModel* link)
{
	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();

	// consider all rigidly connected parent links as well
	const robot_model::LinkModel* parent = robot_model::RobotModel::getRigidlyConnectedParentLinkModel(link);
	if (parent != link)  // transform pose into pose suitable to place parent
		pose = pose * robot_state.getGlobalLinkTransform(link).inverse() * robot_state.getGlobalLinkTransform(parent);

	// place link at given pose
	robot_state.updateStateWithLinkAt(parent, pose);
	robot_state.updateCollisionBodyTransforms();

	// disable collision checking for parent links (except links fixed to root)
	auto& acm = scene->getAllowedCollisionMatrixNonConst();
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
	collision_detection::CollisionResult res;
	scene->checkCollision(req, res, robot_state, acm);
	return res.collision;
}

bool validateEEF(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                 const moveit::core::JointModelGroup*& jmg, std::string* msg)
{
	try {
		const std::string& eef = props.get<std::string>("eef");
		if (!robot_model->hasEndEffector(eef)) {
			if (msg) *msg = "Unknown end effector: " + eef;
			return false;
		} else
			jmg = robot_model->getEndEffector(eef);
	} catch (const Property::undefined&) {
	}
	return true;
}
bool validateGroup(const PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                   const moveit::core::JointModelGroup* eef_jmg,
                   const moveit::core::JointModelGroup*& jmg, std::string* msg)
{
	try {
		const std::string& group = props.get<std::string>("group");
		if (!(jmg = robot_model->getJointModelGroup(group))) {
			if (msg) *msg = "Unknown group: " + group;
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

} // anonymous namespace

void ComputeIK::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	InitStageException errors;
	try { WrapperBase::init(robot_model); } catch (InitStageException &e) { errors.append(e); }

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

	if (errors) throw errors;
}

void ComputeIK::onNewSolution(const SolutionBase &s)
{
	assert(s.start() && s.end());
	assert(s.start()->scene() == s.end()->scene()); // wrapped child should be a generator
	planning_scene::PlanningScenePtr sandbox_scene = s.start()->scene()->diff();

	// enforced initialization from interface ensures that new target_pose is read
	properties().performInitFrom(INTERFACE, s.start()->properties(), true);
	const auto& props = properties();

	const auto& robot_model = sandbox_scene->getRobotModel();
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

	// extract target_pose
	geometry_msgs::PoseStamped target_pose_msg = props.get<geometry_msgs::PoseStamped>("target_pose");
	if (target_pose_msg.header.frame_id.empty())  // if not provided, assume planning frame
		target_pose_msg.header.frame_id = sandbox_scene->getPlanningFrame();

	Eigen::Affine3d target_pose;
	tf::poseMsgToEigen(target_pose_msg.pose, target_pose);
	if (target_pose_msg.header.frame_id != sandbox_scene->getPlanningFrame()) {
		if (!sandbox_scene->knowsFrameTransform(target_pose_msg.header.frame_id)) {
			ROS_WARN_STREAM_NAMED("ComputeIK", "Unknown reference frame for target pose: " << target_pose_msg.header.frame_id);
			return;
		}
		// transform target_pose w.r.t. planning frame
		target_pose = sandbox_scene->getFrameTransform(target_pose_msg.header.frame_id) * target_pose;
	}

	// determine IK link from ik_frame
	const robot_model::LinkModel* link = nullptr;
	geometry_msgs::PoseStamped ik_pose_msg;
	const boost::any& value = props.get("ik_frame");
	if (value.empty()) { // property undefined
		//  determine IK link from eef/group
		if (!(link = eef_jmg ? robot_model->getLinkModel(eef_jmg->getEndEffectorParentGroup().second)
		                     : jmg->getOnlyOneEndEffectorTip())) {
			ROS_WARN_STREAM_NAMED("ComputeIK", "Failed to derive IK target link");
			return;
		}
		ik_pose_msg.header.frame_id = link->getName();
		ik_pose_msg.pose.orientation.w = 1.0;
	} else {
		ik_pose_msg = boost::any_cast<geometry_msgs::PoseStamped>(value);
		Eigen::Affine3d ik_pose;
		tf::poseMsgToEigen(ik_pose_msg.pose, ik_pose);
		if (!(link = robot_model->getLinkModel(ik_pose_msg.header.frame_id))) {
			ROS_WARN_STREAM_NAMED("ComputeIK", "Unknown link: " << ik_pose_msg.header.frame_id);
			return;
		}
		// transform target pose such that ik frame will reach there if link does
		target_pose = target_pose * ik_pose.inverse();
	}

	// validate placed link for collisions
	bool colliding = isTargetPoseColliding(sandbox_scene, target_pose, link);
	if (colliding && !storeFailures()) {
		ROS_ERROR("eef in collision");
		return;
	}
	robot_state::RobotState& sandbox_state = sandbox_scene->getCurrentStateNonConst();

	// markers used for failures
	std::deque<visualization_msgs::Marker> failure_markers;
	// frames at target pose and ik frame
	rviz_marker_tools::appendFrame(failure_markers, target_pose_msg, 0.1, "ik frame");
	rviz_marker_tools::appendFrame(failure_markers, ik_pose_msg, 0.1, "ik frame");
	// visualize placed end-effector
	auto appender = [&failure_markers](visualization_msgs::Marker& marker, const std::string& name) {
		marker.ns = "ik target";
		marker.color.a *= 0.5;
		failure_markers.push_back(marker);
	};
	const auto& link_to_visualize = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link)
	                                ->getParentJointModel()->getDescendantLinkModels();
	if (colliding) {
		SubTrajectory solution;
		generateCollisionMarkers(sandbox_state, appender, link_to_visualize);
		std::copy(failure_markers.begin(), failure_markers.end(), std::back_inserter(solution.markers()));
		solution.setCost(std::numeric_limits<double>::infinity());  // mark solution as failure
		solution.setName("eef in collision");
		spawn(InterfaceState(sandbox_scene), std::move(solution));
		return;
	} else
		generateVisualMarkers(sandbox_state, appender, link_to_visualize);


	// determine joint values of robot pose to compare IK solution with for costs
	std::vector<double> compare_pose;
	const std::string &compare_pose_name = props.get<std::string>("default_pose");
	if (!compare_pose_name.empty()) {
		robot_state::RobotState compare_state(robot_model);
		compare_state.setToDefaultValues(jmg, compare_pose_name);
		compare_state.copyJointGroupPositions(jmg, compare_pose);
	} else
		sandbox_scene->getCurrentState().copyJointGroupPositions(jmg, compare_pose);

	IKSolutions ik_solutions;
	bool ignore_collisions = props.get<bool>("ignore_collisions");
	auto isValid = [sandbox_scene, ignore_collisions, &ik_solutions]
	               (robot_state::RobotState* state, const robot_model::JointModelGroup* jmg, const double* joint_positions) {
		for (const auto& sol : ik_solutions){
			if (jmg->distance(joint_positions, sol.data()) < 0.1)
				return false; // too close to already found solution
		}
		state->setJointGroupPositions(jmg, joint_positions);
		ik_solutions.emplace_back();
		state->copyJointGroupPositions(jmg, ik_solutions.back());

		return ignore_collisions || !sandbox_scene->isStateColliding(*state, jmg->getName());
	};

	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	bool tried_current_state_as_seed = false;

	double remaining_time = props.get<double>("timeout");
	auto start_time = std::chrono::steady_clock::now();
	while (ik_solutions.size() < max_ik_solutions && remaining_time > 0) {
		if(tried_current_state_as_seed)
			sandbox_state.setToRandomPositions(jmg);
		tried_current_state_as_seed= true;

		size_t previous = ik_solutions.size();
		bool succeeded = sandbox_state.setFromIK(jmg, target_pose, link->getName(), 1, remaining_time, isValid);

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;

		if (succeeded || (storeFailures() && ik_solutions.size() > previous)) {
			// create a new scene for each solution as they will have different robot states
			planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();
			SubTrajectory solution;

			// frames at target pose and ik frame
			rviz_marker_tools::appendFrame(solution.markers(), target_pose_msg, 0.1, "ik frame");
			rviz_marker_tools::appendFrame(solution.markers(), ik_pose_msg, 0.1, "ik frame");

			if (succeeded)
				// compute cost as distance to compare_pose
				solution.setCost(s.cost() + jmg->distance(ik_solutions.back().data(), compare_pose.data()));
			else // found an IK solution, but this was not valid
				solution.setCost(std::numeric_limits<double>::infinity());

			// set scene's robot state
			robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
			robot_state.setJointGroupPositions(jmg, ik_solutions.back().data());
			robot_state.update();

			spawn(InterfaceState(scene), std::move(solution));
		}

		if (!succeeded && max_ik_solutions == 1)
			break;  // first and only attempt failed
	}

	if (ik_solutions.empty() && storeFailures()) {  // failed to find any solution
		planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();
		SubTrajectory solution;

		// mark solution as invalid
		solution.setCost(std::numeric_limits<double>::infinity());
		solution.setName("no IK found");

		// ik target link placement
		std::copy(failure_markers.begin(), failure_markers.end(), std::back_inserter(solution.markers()));

		spawn(InterfaceState(scene), std::move(solution));
	}
}

} } }
