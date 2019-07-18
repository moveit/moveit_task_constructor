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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <QObject>
#include <boost/thread/mutex.hpp>

class QColor;

namespace Ogre {
class SceneNode;
}

namespace rviz {
class Display;
class DisplayContext;
class Robot;
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EnumProperty;
class EditableEnumProperty;
class ColorProperty;
class PanelDockWidget;
}

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel)
}
}
namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}
namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(RobotStateVisualization)
MOVEIT_CLASS_FORWARD(TaskSolutionVisualization)
MOVEIT_CLASS_FORWARD(PlanningSceneRender)
MOVEIT_CLASS_FORWARD(DisplaySolution)

class TaskSolutionPanel;
class MarkerVisualizationProperty;
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

	planning_scene::PlanningSceneConstPtr getScene() const { return scene_; }
	void showTrajectory(const moveit_task_constructor_msgs::Solution& msg);
	void showTrajectory(const moveit_rviz_plugin::DisplaySolutionPtr& s, bool lock);
	void unlock();

	void clearMarkers();
	void addMarkers(const moveit_rviz_plugin::DisplaySolutionPtr& s);

public Q_SLOTS:
	void interruptCurrentDisplay();

private Q_SLOTS:
	void onAllAtOnceChanged(bool);

	// trajectory property slots
	void changedRobotVisualEnabled();
	void changedRobotCollisionEnabled();
	void changedRobotAlpha();
	void changedLoopDisplay();
	void changedTrail();
	void changedRobotColor();
	void enabledRobotColor();
	void changedAttachedBodyColor();
	void sliderPanelVisibilityChange(bool enable);

	// planning scene property slots
	void changedSceneEnabled();
	void renderCurrentScene();

Q_SIGNALS:
	void activeStageChanged(size_t);

protected:
	void setVisibility();  ///< set visibility of main scene node
	void setVisibility(Ogre::SceneNode* node, Ogre::SceneNode* parent, bool visible);
	float getStateDisplayTime();
	void clearTrail();
	void renderCurrentWayPoint();
	void renderWayPoint(size_t index, int previous_index);
	void renderPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

	// render the planning scene
	PlanningSceneRenderPtr scene_render_;
	// render the robot
	RobotStateVisualizationPtr robot_render_;
	// render markers
	MarkerVisualizationProperty* marker_visual_;

	// Handle colouring of robot
	void setRobotColor(rviz::Robot* robot, const QColor& color);
	void unsetRobotColor(rviz::Robot* robot);

	DisplaySolutionPtr displaying_solution_;
	DisplaySolutionPtr next_solution_to_display_;
	std::vector<rviz::Robot*> trail_;
	bool animating_ = false;  // auto-progressing the current waypoint?
	bool drop_displaying_solution_ = false;
	bool locked_ = false;
	int current_state_ = -1;
	float current_state_time_;
	boost::mutex display_solution_mutex_;

	planning_scene::PlanningScenePtr scene_;

	// Pointers from parent display that we save
	rviz::Display* display_;  // the parent display that this class populates
	Ogre::SceneNode* parent_scene_node_;  // parent scene node provided by display
	Ogre::SceneNode* main_scene_node_;  // to be added/removed to/from scene_node_
	Ogre::SceneNode* trail_scene_node_;  // to be added/removed to/from scene_node_
	rviz::DisplayContext* context_;
	TaskSolutionPanel* slider_panel_ = nullptr;
	rviz::PanelDockWidget* slider_dock_panel_ = nullptr;
	bool slider_panel_was_visible_ = false;

	// Trajectory Properties
	rviz::Property* robot_property_;
	rviz::BoolProperty* robot_visual_enabled_property_;
	rviz::BoolProperty* robot_collision_enabled_property_;
	rviz::FloatProperty* robot_alpha_property_;
	rviz::ColorProperty* robot_color_property_;
	rviz::BoolProperty* enable_robot_color_property_;

	rviz::EditableEnumProperty* state_display_time_property_;
	rviz::BoolProperty* loop_display_property_;
	rviz::BoolProperty* trail_display_property_;
	rviz::BoolProperty* interrupt_display_property_;
	rviz::IntProperty* trail_step_size_property_;

	// PlanningScene Properties
	rviz::BoolProperty* scene_enabled_property_;
	rviz::FloatProperty* scene_alpha_property_;
	rviz::ColorProperty* scene_color_property_;
	rviz::ColorProperty* attached_body_color_property_;
	rviz::EnumProperty* octree_render_property_;
	rviz::EnumProperty* octree_coloring_property_;
};

}  // namespace moveit_rviz_plugin
