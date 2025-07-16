/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Sherbrooke University
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
/* Authors: Captain Yoshi */

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit {
namespace task_constructor {
namespace stages {

/** no-op stage, which doesn't modify the interface state nor adds a trajectory.
 *  However, it can be used to store custom stage properties,
 *  which in turn can be queried post-planning to steer the execution.
 */

class NoOp : public PropagatingEitherWay
{
public:
	NoOp(const std::string& name = "no-op") : PropagatingEitherWay(name){};

private:
	bool compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& /*trajectory*/,
	             Interface::Direction /*dir*/) override {
		scene = state.scene()->diff();
		return true;
	};
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
