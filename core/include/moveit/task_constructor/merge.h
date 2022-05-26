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

/* Authors: Luca Lach, Robert Haschke */

#pragma once

#include <moveit/task_constructor/storage.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_parameterization.h>

namespace moveit {
namespace task_constructor {

/** Create a new JointModelGroup comprising all joints of the given groups
 *
 *  Throws if there are any duplicate, active joints in the groups */
moveit::core::JointModelGroup* merge(const std::vector<const moveit::core::JointModelGroup*>& groups);

/** merge all sub trajectories into a single RobotTrajectory for parallel execution
 *
 * As the RobotTrajectory maintains a pointer to the underlying JointModelGroup
 * (to know about the involved joint names), a merged JointModelGroup needs to be passed
 * or created on the fly. This JMG needs to stay alive during the lifetime of the trajectory.
 * For now, only the trajectory path is considered. Timings, velocities, etc. are ignored.
 */
robot_trajectory::RobotTrajectoryPtr
merge(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
      const moveit::core::RobotState& base_state, moveit::core::JointModelGroup*& merged_group,
      const trajectory_processing::TimeParameterization& time_parameterization);
}  // namespace task_constructor
}  // namespace moveit
