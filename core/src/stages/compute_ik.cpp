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

namespace {
bool isValid(planning_scene::PlanningSceneConstPtr scene,
             bool ignore_collisions,
             std::vector< std::vector<double> >* found_solutions,
             robot_state::RobotState* state,
             const robot_model::JointModelGroup* jmg,
             const double* joint_positions){
	for(const auto& sol : *found_solutions ){
		if( jmg->distance(joint_positions, sol.data()) < 0.1 )
			return false; // to close at already found solutions
	}

	state->setJointGroupPositions(jmg, joint_positions);
	state->update();
	found_solutions->emplace_back();
	state->copyJointGroupPositions(jmg, found_solutions->back());

	if (ignore_collisions)
		return true;

	if (scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()))
		return false;

	return true;
}

} // anonymous namespace

void ComputeIK::onNewSolution(const SolutionBase &s)
{
	assert(s.start() && s.end());
	assert(s.start()->scene() == s.end()->scene()); // wrapped child should be a generator
	planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();

	// enforced initialization from interface ensures that new target_pose is read
	properties().performInitFrom(INTERFACE, s.start()->properties(), true);
	const auto& props = properties();

	const std::string& eef = props.get<std::string>("eef");
	assert(scene->getRobotModel()->hasEndEffector(eef) && "The specified end effector is not defined in the srdf");

	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	const auto& robot_model = robot_state.getRobotModel();

	const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
	const std::string& link_name = jmg->getEndEffectorParentGroup().second;
	const std::string& group = props.get<std::string>("group");
	jmg = group.empty() ? robot_model->getJointModelGroup(jmg->getEndEffectorParentGroup().first)
	                    : robot_model->getJointModelGroup(group);

	// compute target pose w.r.t. link_name
	geometry_msgs::PoseStamped target_pose_msg = props.get<geometry_msgs::PoseStamped>("target_pose");
	Eigen::Affine3d target_pose;
	tf::poseMsgToEigen(target_pose_msg.pose, target_pose);
	if (!target_pose_msg.header.frame_id.empty() && target_pose_msg.header.frame_id != link_name) {
		const Eigen::Affine3d& ref_pose = scene->getFrameTransform(props.get<std::string>(target_pose_msg.header.frame_id));
		if(ref_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			ROS_WARN("requested reference frame '%s' for target pose does not exist", target_pose_msg.header.frame_id.c_str());
		const Eigen::Affine3d& link_pose = scene->getFrameTransform(link_name);
		if(link_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			ROS_WARN("requested link frame '%s' does not exist", link_name.c_str());

		// transform target pose such that the link frame will reach there
		target_pose = target_pose * ref_pose.inverse() * link_pose;
	}

	// determine joint values of robot pose to compare IK solution with for costs
	std::vector<double> compare_pose;
	const std::string &compare_pose_name = props.get<std::string>("default_pose");
	if (!compare_pose_name.empty()) {
		robot_state::RobotState compare_state(robot_state);
		compare_state.setToDefaultValues(jmg, compare_pose_name);
		compare_state.copyJointGroupPositions(jmg, compare_pose);
	} else
		robot_state.copyJointGroupPositions(jmg, compare_pose);

	const moveit::core::GroupStateValidityCallbackFn is_valid =
		std::bind(&isValid,
	             scene,
	             props.get<bool>("ignore_collisions"),
	             &previous_solutions_,
	             std::placeholders::_1,
	             std::placeholders::_2,
	             std::placeholders::_3);

	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	tried_current_state_as_seed_= false;
	previous_solutions_.clear();

	double remaining_time = props.get<double>("timeout");
	auto start_time = std::chrono::steady_clock::now();
	while (previous_solutions_.size() < max_ik_solutions && remaining_time > 0) {
		if(tried_current_state_as_seed_)
			robot_state.setToRandomPositions(jmg);
		tried_current_state_as_seed_= true;

		bool succeeded = robot_state.setFromIK(jmg, target_pose, link_name, 1, remaining_time, is_valid);

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;

		if (succeeded)
			spawn(InterfaceState(scene), s.cost() + jmg->distance(previous_solutions_.back().data(), compare_pose.data()));
		else if (max_ik_solutions == 1)
			break; // first and only attempt failed
	}
}

} } }
