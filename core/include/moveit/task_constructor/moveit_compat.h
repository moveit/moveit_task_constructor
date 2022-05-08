/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Hamburg University
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

/* Author: Michael 'v4hn' Goerner
   Desc:   Provide test for MoveIt features
*/

#pragma once

#include <moveit/version.h>
#include <moveit/macros/class_forward.h>

#define MOVEIT_VERSION_GE(major, minor, patch)                                                \
	(MOVEIT_VERSION_MAJOR * 1'000'000 + MOVEIT_VERSION_MINOR * 1'000 + MOVEIT_VERSION_PATCH >= \
	 major * 1'000'000 + minor * 1'000 + patch)

// use collision env instead of collision robot/world
#define MOVEIT_HAS_COLLISION_ENV MOVEIT_VERSION_GE(1, 1, 0)

// cartesian interpolator got separated from RobotState at some point
#define MOVEIT_HAS_CARTESIAN_INTERPOLATOR MOVEIT_VERSION_GE(1, 1, 0)

// isEmpty got split off into its own header
#define MOVEIT_HAS_MESSAGE_CHECKS MOVEIT_VERSION_GE(1, 1, 0)

// use object shape poses relative to a single object pose
#define MOVEIT_HAS_OBJECT_POSE MOVEIT_VERSION_GE(1, 1, 6)

#define MOVEIT_HAS_STATE_RIGID_PARENT_LINK MOVEIT_VERSION_GE(1, 1, 6)

#if !MOVEIT_VERSION_GE(1, 1, 9)
// the pointers are not yet available
namespace trajectory_processing {
MOVEIT_CLASS_FORWARD(TimeParameterization);
}
#endif
