//
// Created by llach on 24.11.17.
//

#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <chrono>
#include <functional>
#include <ros/console.h>

namespace moveit { namespace task_constructor { namespace stages {

ComputeIK::ComputeIK(const std::string &name, Stage::pointer &&child)
   : Wrapper(name, std::move(child))
{
	auto& p = properties();
	p.declare<double>("timeout", 1.0);
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("group", "", "name of active group (derived from eef if not provided)");
	p.declare<std::string>("default_pose", "", "default joint pose of active group (defines cost of IK)");
	p.declare<uint32_t>("max_ik_solutions", 1);
	p.declare<bool>("ignore_collisions", false);

	// target_pose is read from the interface
	p.declare<geometry_msgs::PoseStamped>("target_pose");
	p.configureInitFrom(Stage::INTERFACE, {"target_pose"});
}

void ComputeIK::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

void ComputeIK::setEndEffector(const std::string &eef){
	setProperty("eef", eef);
}

void ComputeIK::setTargetPose(const geometry_msgs::PoseStamped &pose)
{
	setProperty("target_pose", pose);
}

void ComputeIK::setTargetPose(const Eigen::Affine3d &pose, const std::string &link)
{
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
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
bool isValid(planning_scene::PlanningSceneConstPtr scene,
             bool ignore_collisions,
             IKSolutions* ik_solutions,
             robot_state::RobotState* state,
             const robot_model::JointModelGroup* jmg,
             const double* joint_positions){
	for (const auto& sol : *ik_solutions){
		if (jmg->distance(joint_positions, sol.data()) < 0.1)
			return false; // to close to already found solution
	}
	state->setJointGroupPositions(jmg, joint_positions);
	ik_solutions->emplace_back();
	state->copyJointGroupPositions(jmg, ik_solutions->back());

	return ignore_collisions || !scene->isStateColliding(*state, jmg->getName());
}

bool isTargetPoseColliding(const planning_scene::PlanningSceneConstPtr& scene,
                           const robot_model::JointModelGroup* jmg,
                           const Eigen::Affine3d &pose,
                           const std::string &link_name)
{
	planning_scene::PlanningScenePtr sandbox_scene = scene->diff();
	robot_state::RobotState& robot_state = sandbox_scene->getCurrentStateNonConst();
	robot_state.updateStateWithLinkAt(link_name, pose, true);

	// disable collision checking for parent links (except fixed links)
	auto& acm = sandbox_scene->getAllowedCollisionMatrixNonConst();
	const robot_model::LinkModel* link = robot_state.getLinkModel(link_name);
	const robot_model::LinkModel* parent_link = link->getParentLinkModel();
	const robot_model::JointModel* joint = link->getParentJointModel();

	// keep links rigidly attached to link_name
	std::vector<const std::string*> fixed_eef_links;
	while (parent_link && joint->getType() == robot_model::JointModel::FIXED) {
		fixed_eef_links.push_back(&parent_link->getName());
		link = parent_link;
		joint = link->getParentJointModel();
		parent_link = joint->getParentLinkModel();
	}
	// now parent_link is the first link that is not rigidly attached to link_name

	std::vector<const std::string*> pending_links;
	while (parent_link) {
		pending_links.push_back(&parent_link->getName());
		link = parent_link;
		joint = link->getParentJointModel();
		parent_link = joint->getParentLinkModel();

		if (joint->getType() != robot_model::JointModel::FIXED) {
			for (const std::string* name : pending_links)
				acm.setDefaultEntry(*name, true);
			pending_links.clear();
		}
	}

	// check collision with the world using the padded version
	collision_detection::CollisionRequest req;
	req.group_name = jmg->getName();
	collision_detection::CollisionResult res;
	scene->getCollisionWorld()->checkRobotCollision(req, res, *scene->getCollisionRobot(), robot_state, acm);
	return res.collision;
}

} // anonymous namespace

void ComputeIK::onNewSolution(const SolutionBase &s)
{
	assert(s.start() && s.end());
	assert(s.start()->scene() == s.end()->scene()); // wrapped child should be a generator
	planning_scene::PlanningScenePtr sandbox_scene = s.start()->scene()->diff();

	// enforced initialization from interface ensures that new target_pose is read
	properties().performInitFrom(INTERFACE, s.start()->properties(), true);
	const auto& props = properties();

	const std::string& eef = props.get<std::string>("eef");
	assert(sandbox_scene->getRobotModel()->hasEndEffector(eef) && "The specified end effector is not defined in the srdf");

	const auto& robot_model = sandbox_scene->getRobotModel();
	const moveit::core::JointModelGroup* eef_jmg = robot_model->getEndEffector(eef);
	const std::string& link_name = eef_jmg->getEndEffectorParentGroup().second;

	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = group.empty() ? robot_model->getJointModelGroup(eef_jmg->getEndEffectorParentGroup().first)
	                                                         : robot_model->getJointModelGroup(group);

	// compute target pose w.r.t. link_name
	geometry_msgs::PoseStamped target_pose_msg = props.get<geometry_msgs::PoseStamped>("target_pose");
	Eigen::Affine3d target_pose;
	tf::poseMsgToEigen(target_pose_msg.pose, target_pose);
	if (!target_pose_msg.header.frame_id.empty() && target_pose_msg.header.frame_id != link_name) {
		const Eigen::Affine3d& ref_pose = sandbox_scene->getFrameTransform(props.get<std::string>(target_pose_msg.header.frame_id));
		if(ref_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			ROS_WARN("requested reference frame '%s' for target pose does not exist", target_pose_msg.header.frame_id.c_str());
		const Eigen::Affine3d& link_pose = sandbox_scene->getFrameTransform(link_name);
		if(link_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			ROS_WARN("requested link frame '%s' does not exist", link_name.c_str());

		// transform target pose such that the link frame will reach there
		target_pose = target_pose * ref_pose.inverse() * link_pose;
	}

	// validate ee group for collision
	if (isTargetPoseColliding(sandbox_scene, eef_jmg, target_pose, link_name)) {
		ROS_ERROR("eeg in collision");
		return;
	}

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
	robot_state::RobotState& sandbox_state = sandbox_scene->getCurrentStateNonConst();

	const moveit::core::GroupStateValidityCallbackFn is_valid =
		std::bind(&isValid,
	             sandbox_scene,
	             props.get<bool>("ignore_collisions"),
	             &ik_solutions,
	             std::placeholders::_1,
	             std::placeholders::_2,
	             std::placeholders::_3);

	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	bool tried_current_state_as_seed = false;

	double remaining_time = props.get<double>("timeout");
	auto start_time = std::chrono::steady_clock::now();
	while (ik_solutions.size() < max_ik_solutions && remaining_time > 0) {
		if(tried_current_state_as_seed)
			sandbox_state.setToRandomPositions(jmg);
		tried_current_state_as_seed= true;

		bool succeeded = sandbox_state.setFromIK(jmg, target_pose, link_name, 1, remaining_time, is_valid);

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;

		if (succeeded) {
			// create a new scene for each solution as they will have different robot states
			planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();
			robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
			robot_state.setJointGroupPositions(jmg, ik_solutions.back().data());

			spawn(InterfaceState(scene), s.cost() + jmg->distance(ik_solutions.back().data(), compare_pose.data()));
		} else if (max_ik_solutions == 1)
			break; // first and only attempt failed
	}
}

} } }
