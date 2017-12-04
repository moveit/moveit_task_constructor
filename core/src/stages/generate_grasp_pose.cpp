#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

#include <chrono>
#include <functional>

namespace moveit { namespace task_constructor { namespace stages {

GenerateGraspPose::GenerateGraspPose(std::string name)
: Generator(name)
{
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("link", "", "name of link used for IK");
	p.declare<std::string>("object");
	p.declare<std::string>("grasp_pose");
	p.declare<double>("timeout", 0.1);
	p.declare<uint32_t>("max_ik_solutions", 1);
	p.declare<double>("grasp_offset", 0.0);
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
	p.declare<bool>("ignore_collisions", false);
}

void GenerateGraspPose::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Generator::init(scene);
	scene_ = scene;
}

void GenerateGraspPose::setGroup(std::string group){
	setProperty("group", group);
}

void GenerateGraspPose::setLink(std::string ik_link){
	setProperty("link", ik_link);
}

void GenerateGraspPose::setEndEffector(std::string eef){
	setProperty("eef", eef);
}

void GenerateGraspPose::setGripperGraspPose(std::string pose_name){
	setProperty("grasp_pose", pose_name);
}

void GenerateGraspPose::setObject(std::string object){
	setProperty("object", object);
}

void GenerateGraspPose::setGraspOffset(double offset){
	setProperty("grasp_offset", offset);
}

void GenerateGraspPose::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

void GenerateGraspPose::setAngleDelta(double delta){
	setProperty("angle_delta", delta);
}

void GenerateGraspPose::setMaxIKSolutions(uint32_t n){
	setProperty("max_ik_solutions", n);
}

void GenerateGraspPose::setIgnoreCollisions(bool flag)
{
	setProperty("ignore_collisions", flag);
}

bool GenerateGraspPose::canCompute() const{
	return current_angle_ < 2*M_PI && current_angle_ > -2*M_PI;
}

namespace {
	bool isValid(planning_scene::PlanningSceneConstPtr scene,
	             bool ignore_collisions,
	             std::vector< std::vector<double> >* old_solutions,
	             robot_state::RobotState* state,
	             const robot_model::JointModelGroup* jmg,
	             const double* joint_positions){
		for( std::vector<double> sol : *old_solutions ){
			if( jmg->distance(joint_positions, sol.data()) < 0.1 ){
				return false;
			}
		}

		if(ignore_collisions)
			return true;

		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		if( scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()) ){
			old_solutions->emplace_back();
			state->copyJointGroupPositions(jmg, old_solutions->back());
			return false;
		}
		return true;
	}
}

bool GenerateGraspPose::compute(){
	const auto& props = properties();
	double remaining_time = props.get<double>("timeout");

	const std::string& eef = props.get<std::string>("eef");
	const std::string& group = props.get<std::string>("group");

	assert(scene_->getRobotModel()->hasEndEffector(eef) && "The specified end effector is not defined in the srdf");

	planning_scene::PlanningScenePtr grasp_scene = scene_->diff();
	robot_state::RobotState &grasp_state = grasp_scene->getCurrentStateNonConst();

	const moveit::core::JointModelGroup* jmg_eef= grasp_state.getRobotModel()->getEndEffector(eef);

	const moveit::core::JointModelGroup* jmg_active= group.empty()
		? grasp_state.getJointModelGroup(jmg_eef->getEndEffectorParentGroup().first)
		: grasp_state.getJointModelGroup(group);

	std::string link = props.get<std::string>("link");
	if (link.empty()) link = jmg_eef->getEndEffectorParentGroup().second;

	const std::string& grasp_pose_name = props.get<std::string>("grasp_pose");
	if(!grasp_pose_name.empty()){
		grasp_state.setToDefaultValues(jmg_eef, grasp_pose_name);
	}

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			scene_,
			props.get<bool>("ignore_collisions"),
			&previous_solutions_,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);

	geometry_msgs::Pose object_pose, grasp_pose;
	const Eigen::Affine3d object_pose_eigen= scene_->getFrameTransform(props.get<std::string>("object"));
	if(object_pose_eigen.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	tf::poseEigenToMsg(object_pose_eigen, object_pose);

	double grasp_offset = props.get<double>("grasp_offset");
	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	while( canCompute() ){
		if( remaining_time <= 0.0 || (max_ik_solutions != 0 && previous_solutions_.size() >= max_ik_solutions)){
			std::cout << "computed angle " << current_angle_
			          << " with " << previous_solutions_.size()
			          << " cached ik solutions"
			          << " and " << remaining_time << "s left" << std::endl;
			current_angle_+= props.get<double>("angle_delta");
			remaining_time = props.get<double>("timeout");
			tried_current_state_as_seed_= false;
			previous_solutions_.clear();
			continue;
		}

		grasp_pose= object_pose;

		grasp_pose.position.x-= grasp_offset*cos(current_angle_);
		grasp_pose.position.y-= grasp_offset*sin(current_angle_);
		grasp_pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, current_angle_);

		if(tried_current_state_as_seed_)
			grasp_state.setToRandomPositions(jmg_active);
		tried_current_state_as_seed_= true;

		auto now= std::chrono::steady_clock::now();
		bool succeeded= grasp_state.setFromIK(jmg_active, grasp_pose, link, 1, remaining_time, is_valid);
		remaining_time-= std::chrono::duration<double>(std::chrono::steady_clock::now()- now).count();

		if(succeeded) {
			previous_solutions_.emplace_back();
			grasp_state.copyJointGroupPositions(jmg_active, previous_solutions_.back());
			spawn(InterfaceState(grasp_scene));
			return true;
		}
	}

	return false;
}

} } }
