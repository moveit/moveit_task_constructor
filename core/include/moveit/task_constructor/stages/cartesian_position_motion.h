/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Authors: Michael Goerner
   Desc:    Stage to plan linear cartesian motions
*/

#pragma once

#include <moveit/task_constructor/stage.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

namespace moveit {
namespace planning_interface { MOVEIT_CLASS_FORWARD(MoveGroupInterface) }
namespace core { MOVEIT_CLASS_FORWARD(RobotState) }
}

namespace moveit { namespace task_constructor { namespace stages {

class CartesianPositionMotion : public PropagatingEitherWay {
public:
	CartesianPositionMotion(std::string name);

	virtual bool computeForward(const InterfaceState &from) override;
	virtual bool computeBackward(const InterfaceState &to) override;

	void setGroup(std::string group);
	void setLink(std::string link);

	void setMinDistance(double distance);
	void setMaxDistance(double distance);
	void setMinMaxDistance(double min_distance, double max_distance);

	void towards(geometry_msgs::PointStamped goal);
	void along(geometry_msgs::Vector3Stamped direction);

	void setCartesianStepSize(double distance);

protected:
	std::string group_;

	std::string link_;

	double min_distance_;
	double max_distance_;

	enum {
		MODE_ALONG= 1,
		MODE_TOWARDS= 2
	} mode_;

	geometry_msgs::PointStamped towards_;
	geometry_msgs::Vector3Stamped along_;

	double step_size_;
};

} } }
