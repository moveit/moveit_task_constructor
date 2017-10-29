/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Wraps a task_solution_visualization playback class for Rviz into a stand alone display
*/

#ifndef MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TASK_SOLUTION_DISPLAY
#define MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TASK_SOLUTION_DISPLAY

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <moveit/macros/class_forward.h>
#endif

namespace rviz
{
class StringProperty;
}

namespace moveit { namespace core { MOVEIT_CLASS_FORWARD(RobotModel) } }
namespace rdf_loader { MOVEIT_CLASS_FORWARD(RDFLoader) }

namespace moveit_rviz_plugin
{

MOVEIT_CLASS_FORWARD(TaskSolutionVisualization)
class TaskDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TaskDisplay();

  virtual ~TaskDisplay();

  void loadRobotModel();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  // overrides from Display
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  void setName(const QString& name);

private Q_SLOTS:
  /**
   * \brief Slot Event Functions
   */
  void changedRobotDescription();

protected:
  // The trajectory playback component
  TaskSolutionVisualizationPtr trajectory_visual_;

  // Load robot model
  rdf_loader::RDFLoaderPtr rdf_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  bool load_robot_model_;  // for delayed robot initialization

  // Properties
  rviz::StringProperty* robot_description_property_;
};

}  // namespace moveit_rviz_plugin

#endif
