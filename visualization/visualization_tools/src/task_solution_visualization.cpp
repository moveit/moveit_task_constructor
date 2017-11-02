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

/* Author: Dave Coleman, Robert Haschke */

#include <moveit_task_constructor/visualization_tools/task_solution_visualization.h>
#include <moveit_task_constructor/visualization_tools/task_solution_panel.h>


#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/properties/property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/panel_dock_widget.h>

namespace moveit_rviz_plugin
{
TaskSolutionVisualization::TaskSolutionVisualization(rviz::Property* parent, rviz::Display* display)
  : display_(display)
  , parent_(parent)
{
  display_path_visual_enabled_property_ =
      new rviz::BoolProperty("Show Robot Visual", true, "Indicates whether the geometry of the robot as defined for "
                                                        "visualisation purposes should be displayed",
                             parent, SLOT(changedDisplayPathVisualEnabled()), this);

  display_path_collision_enabled_property_ =
      new rviz::BoolProperty("Show Robot Collision", false, "Indicates whether the geometry of the robot as defined "
                                                            "for collision detection purposes should be displayed",
                             parent, SLOT(changedDisplayPathCollisionEnabled()), this);

  robot_path_alpha_property_ = new rviz::FloatProperty("Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                                                       parent, SLOT(changedRobotPathAlpha()), this);
  robot_path_alpha_property_->setMin(0.0);
  robot_path_alpha_property_->setMax(1.0);

  state_display_time_property_ = new rviz::EditableEnumProperty("State Display Time", "0.05 s",
                                                                "The amount of wall-time to wait in between displaying "
                                                                "states along a received trajectory path",
                                                                parent, SLOT(changedStateDisplayTime()), this);
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");

  loop_display_property_ = new rviz::BoolProperty("Loop Animation", false, "Indicates whether the last received path "
                                                                           "is to be animated in a loop",
                                                  parent, SLOT(changedLoopDisplay()), this);

  trail_display_property_ =
      new rviz::BoolProperty("Show Trail", false, "Show a path trail", parent, SLOT(changedShowTrail()), this);

  trail_step_size_property_ = new rviz::IntProperty("Trail Step Size", 1, "Specifies the step size of the samples "
                                                                          "shown in the trajectory trail.",
                                                    parent, SLOT(changedTrailStepSize()), this);
  trail_step_size_property_->setMin(1);

  interrupt_display_property_ = new rviz::BoolProperty(
        "Interrupt Display", false,
        "Immediately show newly planned trajectory, interrupting the currently displayed one.", parent);

  robot_color_property_ = new rviz::ColorProperty(
        "Robot Color", QColor(150, 50, 150), "The color of the animated robot", parent, SLOT(changedRobotColor()), this);

  enable_robot_color_property_ = new rviz::BoolProperty(
        "Color Enabled", false, "Specifies whether robot coloring is enabled", parent, SLOT(enabledRobotColor()), this);
}

TaskSolutionVisualization::~TaskSolutionVisualization()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();

  display_path_robot_.reset();
  if (trajectory_slider_dock_panel_)
    delete trajectory_slider_dock_panel_;
}

void TaskSolutionVisualization::onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context)
{
  // Save pointers for later use
  scene_node_ = scene_node;
  context_ = context;

  // Load trajectory robot
  display_path_robot_.reset(new RobotStateVisualization(scene_node_, context_, "Solution Trajectory", parent_));
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  if (window_context)
  {
    trajectory_slider_panel_ = new TaskSolutionPanel(window_context->getParentWindow());
    trajectory_slider_dock_panel_ =
        window_context->addPane(display_->getName() + " - Slider", trajectory_slider_panel_);
    trajectory_slider_dock_panel_->setIcon(display_->getIcon());
    connect(trajectory_slider_dock_panel_, SIGNAL(visibilityChanged(bool)), this,
            SLOT(trajectorySliderPanelVisibilityChange(bool)));
    trajectory_slider_panel_->onInitialize();
  }
}

void TaskSolutionVisualization::setName(const QString& name)
{
  if (trajectory_slider_dock_panel_)
    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TaskSolutionVisualization::onRobotModelLoaded(robot_model::RobotModelConstPtr robot_model)
{
  robot_model_ = robot_model;

  // Error check
  if (!robot_model_)
  {
    ROS_ERROR_STREAM_NAMED("task_solution_visualization", "No robot model found");
    return;
  }

  // Load robot state
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // Load rviz robot
  display_path_robot_->load(*robot_model_->getURDF());
  enabledRobotColor();  // force-refresh to account for saved display configuration
}

void TaskSolutionVisualization::reset()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);
}

void TaskSolutionVisualization::clearTrajectoryTrail()
{
  for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
    delete trajectory_trail_[i];
  trajectory_trail_.clear();
}

void TaskSolutionVisualization::changedLoopDisplay()
{
  display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
  if (loop_display_property_->getBool() && trajectory_slider_panel_)
    trajectory_slider_panel_->pauseButton(false);
}

void TaskSolutionVisualization::changedShowTrail()
{
  clearTrajectoryTrail();

  if (!trail_display_property_->getBool())
    return;
  robot_trajectory::RobotTrajectoryPtr t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;
  if (!t)
    return;

  int stepsize = trail_step_size_property_->getInt();
  // always include last trajectory point
  trajectory_trail_.resize((int)std::ceil((t->getWayPointCount() + stepsize - 1) / (float)stepsize));
  for (std::size_t i = 0; i < trajectory_trail_.size(); i++)
  {
    int waypoint_i = std::min(i * stepsize, t->getWayPointCount() - 1);  // limit to last trajectory point
    rviz::Robot* r = new rviz::Robot(scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), NULL);
    r->load(*robot_model_->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool());
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    r->setAlpha(robot_path_alpha_property_->getFloat());
    r->update(PlanningLinkUpdater(t->getWayPointPtr(waypoint_i)));
    if (enable_robot_color_property_->getBool())
      setRobotColor(r, robot_color_property_->getColor());
    r->setVisible(display_->isEnabled() && (!animating_path_ || waypoint_i <= current_state_));
    trajectory_trail_[i] = r;
  }
}

void TaskSolutionVisualization::changedTrailStepSize()
{
  if (trail_display_property_->getBool())
    changedShowTrail();
}

void TaskSolutionVisualization::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
    trajectory_trail_[i]->setAlpha(robot_path_alpha_property_->getFloat());
}

void TaskSolutionVisualization::changedDisplayPathVisualEnabled()
{
  if (display_->isEnabled())
  {
    display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
    display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
      trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
  }
}

void TaskSolutionVisualization::changedStateDisplayTime()
{
}

void TaskSolutionVisualization::changedDisplayPathCollisionEnabled()
{
  if (display_->isEnabled())
  {
    display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
      trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  }
}

void TaskSolutionVisualization::onEnable()
{
  changedRobotPathAlpha();  // set alpha property

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(displaying_trajectory_message_ && animating_path_);
  for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
  {
    trajectory_trail_[i]->setVisualVisible(display_path_visual_enabled_property_->getBool());
    trajectory_trail_[i]->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    trajectory_trail_[i]->setVisible(true);
  }
}

void TaskSolutionVisualization::onDisable()
{
  display_path_robot_->setVisible(false);
  for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
    trajectory_trail_[i]->setVisible(false);
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  if (trajectory_slider_panel_)
    trajectory_slider_panel_->onDisable();
}

void TaskSolutionVisualization::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (current_state_ > 0)
    animating_path_ = false;
}

float TaskSolutionVisualization::getStateDisplayTime()
{
  std::string tm = state_display_time_property_->getStdString();
  if (tm == "REALTIME")
    return -1.0;
  else
  {
    boost::replace_all(tm, "s", "");
    boost::trim(tm);
    float t = 0.05f;
    try
    {
      t = boost::lexical_cast<float>(tm);
    }
    catch (const boost::bad_lexical_cast& ex)
    {
      state_display_time_property_->setStdString("0.05 s");
    }
    return t;
  }
}

void TaskSolutionVisualization::dropTrajectory()
{
  drop_displaying_trajectory_ = true;
}

void TaskSolutionVisualization::update(float wall_dt, float ros_dt)
{
  if (drop_displaying_trajectory_)
  {
    animating_path_ = false;
    displaying_trajectory_message_.reset();
    display_path_robot_->setVisible(false);
    trajectory_slider_panel_->update(0);
    drop_displaying_trajectory_ = false;
  }
  if (!animating_path_)
  {  // finished last animation?
    boost::mutex::scoped_lock lock(update_trajectory_message_);

    // new trajectory available to display?
    if (trajectory_message_to_display_ && !trajectory_message_to_display_->empty())
    {
      animating_path_ = true;
      displaying_trajectory_message_ = trajectory_message_to_display_;
      changedShowTrail();
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->update(trajectory_message_to_display_->getWayPointCount());
    }
    else if (displaying_trajectory_message_)
    {
      if (loop_display_property_->getBool())
      {  // do loop? -> start over too
        animating_path_ = true;
      }
      else if (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible())
      {
        if (trajectory_slider_panel_->getSliderPosition() == (int)displaying_trajectory_message_->getWayPointCount() - 1)
        {  // show the last waypoint if the slider is enabled
          display_path_robot_->update(
                displaying_trajectory_message_->getWayPointPtr(displaying_trajectory_message_->getWayPointCount() - 1));
        }
        else
          animating_path_ = true;
      }
    }
    trajectory_message_to_display_.reset();

    if (animating_path_)
    {
      current_state_ = -1;
      current_state_time_ = std::numeric_limits<float>::infinity();
      display_path_robot_->update(displaying_trajectory_message_->getFirstWayPointPtr());
      display_path_robot_->setVisible(display_->isEnabled());
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->setSliderPosition(0);
    }
  }

  if (animating_path_)
  {
    float tm = getStateDisplayTime();
    if (tm < 0.0)  // if we should use realtime
      tm = displaying_trajectory_message_->getWayPointDurationFromPrevious(current_state_ + 1);
    if (current_state_time_ > tm)
    {
      if (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible() && trajectory_slider_panel_->isPaused())
        current_state_ = trajectory_slider_panel_->getSliderPosition();
      else
        ++current_state_;
      int waypoint_count = displaying_trajectory_message_->getWayPointCount();
      if (current_state_ < waypoint_count)
      {
        if (trajectory_slider_panel_)
          trajectory_slider_panel_->setSliderPosition(current_state_);
        display_path_robot_->update(displaying_trajectory_message_->getWayPointPtr(current_state_));
        for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
          trajectory_trail_[i]->setVisible(
                std::min(waypoint_count - 1, static_cast<int>(i) * trail_step_size_property_->getInt()) <=
                current_state_);
      }
      else
      {
        animating_path_ = false;  // animation finished
        display_path_robot_->setVisible(loop_display_property_->getBool());
        if (!loop_display_property_->getBool() && trajectory_slider_panel_)
          trajectory_slider_panel_->pauseButton(true);
      }
      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void TaskSolutionVisualization::showTrajectory(const moveit_task_constructor::Solution& msg)
{
  // Error check
  if (!robot_state_ || !robot_model_)
  {
    ROS_ERROR_STREAM_NAMED("task_solution_visualization", "No robot state or robot model loaded");
    return;
  }

  if (!msg.model_id.empty() && msg.model_id != robot_model_->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected", msg.model_id.c_str(),
             robot_model_->getName().c_str());

  trajectory_message_to_display_.reset();

  robot_trajectory::RobotTrajectoryPtr t(new robot_trajectory::RobotTrajectory(robot_model_, ""));
  for (std::size_t i = 0; i < msg.sub_trajectory.size(); ++i)
  {
    if (t->empty())
    {
      t->setRobotTrajectoryMsg(*robot_state_, msg.sub_trajectory[i].trajectory);
    }
    else
    {
      robot_trajectory::RobotTrajectory tmp(robot_model_, "");
      tmp.setRobotTrajectoryMsg(t->getLastWayPoint(), msg.sub_trajectory[i].trajectory);
      t->append(tmp, 0.0);
    }
  }

  if (!t->empty())
  {
    boost::mutex::scoped_lock lock(update_trajectory_message_);
    trajectory_message_to_display_.swap(t);
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
}

void TaskSolutionVisualization::changedRobotColor()
{
  if (enable_robot_color_property_->getBool())
    setRobotColor(&(display_path_robot_->getRobot()), robot_color_property_->getColor());
}

void TaskSolutionVisualization::enabledRobotColor()
{
  if (enable_robot_color_property_->getBool())
    setRobotColor(&(display_path_robot_->getRobot()), robot_color_property_->getColor());
  else
    unsetRobotColor(&(display_path_robot_->getRobot()));
}

void TaskSolutionVisualization::unsetRobotColor(rviz::Robot* robot)
{
  for (auto& link : robot->getLinks())
    link.second->unsetColor();
}

void TaskSolutionVisualization::setRobotColor(rviz::Robot* robot, const QColor& color)
{
  for (auto& link : robot->getLinks())
    robot->getLink(link.first)->setColor(color.redF(), color.greenF(), color.blueF());
}

void TaskSolutionVisualization::trajectorySliderPanelVisibilityChange(bool enable)
{
  if (!trajectory_slider_panel_)
    return;

  if (enable)
    trajectory_slider_panel_->onEnable();
  else
    trajectory_slider_panel_->onDisable();
}

}  // namespace moveit_rviz_plugin
