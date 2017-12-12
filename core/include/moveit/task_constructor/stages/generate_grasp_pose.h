// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/stage.h>
#include <Eigen/Geometry>

namespace moveit { namespace task_constructor { namespace stages {

class GenerateGraspPose : public Generator {
public:
	GenerateGraspPose(std::string name);

	void init(const planning_scene::PlanningSceneConstPtr& scene) override;
	bool canCompute() const override;
	bool compute() override;

	void setEndEffector(std::string eef);

	void setGroup(std::string group_name);

	void setGripperGraspPose(std::string pose_name);

	void setObject(std::string object);

	void setTimeout(double timeout);

	void setAngleDelta(double delta);

	void setMaxIKSolutions(uint32_t n);

	void setIgnoreCollisions(bool flag);

	void setGraspFrame(const geometry_msgs::TransformStamped &transform);
	void setGraspFrame(const Eigen::Affine3d& transform, const std::string& link = "");
	template <typename T>
	void setGraspFrame(const T& t, const std::string& link = "") {
		Eigen::Affine3d transform; transform = t;
		setGraspFrame(transform, link);
	}

protected:
	planning_scene::PlanningSceneConstPtr scene_;

	/* temp values */

	double current_angle_ = 0.0;

	bool tried_current_state_as_seed_ = false;

	std::vector< std::vector<double> > previous_solutions_;
};

} } }
