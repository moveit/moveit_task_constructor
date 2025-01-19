/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Authors: Elham Iravani, Robert Haschke
   Desc:    Fix collisions in input scene
*/

#pragma once

#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/stage.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit/collision_detection/collision_common.h>

namespace moveit {
namespace task_constructor {
namespace stages {

class FixCollisionObjects : public PropagatingEitherWay
{
public:
	FixCollisionObjects(const std::string& name = "fix collisions of objects");

	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;

	void setDirection(const geometry_msgs::msg::Vector3& dir) { setProperty("direction", dir); }
	void setMaxPenetration(double penetration) { setProperty("max_penetration", penetration); }

private:
	SubTrajectory fixCollisions(planning_scene::PlanningScene& scene) const;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
