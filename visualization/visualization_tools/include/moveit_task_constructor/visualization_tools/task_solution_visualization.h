/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Dave Coleman */

#ifndef MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TASK_SOLITION_VISUALIZATION
#define MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TASK_SOLUTION_VISUALIZATION

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor/Solution.h>
#include <QObject>
#include <QColor>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <ros/node_handle.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Display;
class DisplayContext;
class Robot;
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class ColorProperty;
class PanelDockWidget;
}

namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotModel)
MOVEIT_CLASS_FORWARD(RobotState)
} }
namespace robot_trajectory { MOVEIT_CLASS_FORWARD(RobotTrajectory) }

namespace moveit_rviz_plugin
{

MOVEIT_CLASS_FORWARD(RobotStateVisualization)
MOVEIT_CLASS_FORWARD(TaskSolutionVisualization)

class TaskSolutionPanel;
class TaskSolutionVisualization : public QObject
{
  Q_OBJECT

public:
  /**
   * \brief Playback a trajectory from a planned path
   * \param parent - either a rviz::Display or rviz::Property
   * \param display - the rviz::Display from the parent
   * \return true on success
   */
  TaskSolutionVisualization(rviz::Property* parent, rviz::Display* display);

  virtual ~TaskSolutionVisualization();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context);
  void onRobotModelLoaded(moveit::core::RobotModelConstPtr robot_model);
  void onEnable();
  void onDisable();
  void setName(const QString& name);

  void showTrajectory(const moveit_task_constructor::Solution& msg);
  void dropTrajectory();

public Q_SLOTS:
  void interruptCurrentDisplay();

private Q_SLOTS:

  /**
   * \brief Slot Event Functions
   */
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedLoopDisplay();
  void changedShowTrail();
  void changedTrailStepSize();
  void changedStateDisplayTime();
  void changedRobotColor();
  void enabledRobotColor();
  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  float getStateDisplayTime();
  void clearTrajectoryTrail();

  // Handles actually drawing the robot along motion plans
  RobotStateVisualizationPtr display_path_robot_;

  // Handle colouring of robot
  void setRobotColor(rviz::Robot* robot, const QColor& color);
  void unsetRobotColor(rviz::Robot* robot);

  robot_trajectory::RobotTrajectoryPtr displaying_trajectory_message_;
  robot_trajectory::RobotTrajectoryPtr trajectory_message_to_display_;
  std::vector<rviz::Robot*> trajectory_trail_;
  bool animating_path_ = false;
  bool drop_displaying_trajectory_ = false;
  int current_state_ = -1;
  float current_state_time_;
  boost::mutex update_trajectory_message_;

  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  // Pointers from parent display taht we save
  rviz::Display* display_;  // the parent display that this class populates
  rviz::Property* parent_;
  Ogre::SceneNode* scene_node_;
  rviz::DisplayContext* context_;
  TaskSolutionPanel* trajectory_slider_panel_ = nullptr;
  rviz::PanelDockWidget* trajectory_slider_dock_panel_ = nullptr;

  // Properties
  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::EditableEnumProperty* state_display_time_property_;
  rviz::FloatProperty* robot_path_alpha_property_;
  rviz::BoolProperty* loop_display_property_;
  rviz::BoolProperty* trail_display_property_;
  rviz::BoolProperty* interrupt_display_property_;
  rviz::ColorProperty* robot_color_property_;
  rviz::BoolProperty* enable_robot_color_property_;
  rviz::IntProperty* trail_step_size_property_;
};

}  // namespace moveit_rviz_plugin

#endif
