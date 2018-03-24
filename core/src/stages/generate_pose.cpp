/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

GeneratePose::GeneratePose(const std::string& name)
   : MonitoringGenerator(name)
{
	auto& p = properties();
	p.declare<geometry_msgs::PoseStamped>("pose", "target pose to pass on in spawned states");
}

void GeneratePose::reset()
{
	MonitoringGenerator::reset();
	scenes_.clear();
}

void GeneratePose::onNewSolution(const SolutionBase& s)
{
	scenes_.push_back(s.end()->scene()->diff());
}

bool GeneratePose::canCompute() const {
	return scenes_.size() > 0;
}

bool GeneratePose::compute(){
	const auto& props = properties();

	geometry_msgs::PoseStamped target_pose = props.get<geometry_msgs::PoseStamped>("pose");

	if(scenes_.empty()) throw std::runtime_error("GeneratePose called without checking canCompute.");

	planning_scene::PlanningSceneConstPtr scene = scenes_[0];
	scenes_.pop_front();

	// take care of frame transforms
	const std::string& frame = target_pose.header.frame_id;
	if( !frame.empty() && frame != scene->getPlanningFrame() ){
		if ( !scene->knowsFrameTransform(frame) )
			throw std::runtime_error("GeneratePose does not know frame '" + frame + "'");

		Eigen::Affine3d pose;
		tf::poseMsgToEigen(target_pose.pose, pose);
		pose = scene->getFrameTransform(target_pose.header.frame_id) * pose;
		tf::poseEigenToMsg(pose, target_pose.pose);
		target_pose.header.frame_id = scene->getPlanningFrame();
	}

	InterfaceState state(scene);
	state.properties().set("target_pose", target_pose);

	SubTrajectory trajectory;
	trajectory.setCost(0.0);

	rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "pose frame");

	spawn(std::move(state), std::move(trajectory));
	return true;
}

} } }
