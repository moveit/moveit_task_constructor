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

/* Author: Robert Haschke */

#include <moveit/visualization_tools/display_solution.h>
#include <moveit/visualization_tools/task_solution_visualization.h>
#include <moveit/visualization_tools/marker_visualization.h>
#include <moveit/visualization_tools/task_solution_panel.h>

#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include <OgreSceneNode.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_task_constructor_visualization.task_solution_visualization");

namespace moveit_rviz_plugin {
TaskSolutionVisualization::TaskSolutionVisualization(rviz_common::properties::Property* parent,
                                                     rviz_common::Display* display)
  : display_(display) {
	// trajectory properties
	interrupt_display_property_ = new rviz_common::properties::BoolProperty("Interrupt Display", false,
	                                                                        "Immediately show newly planned trajectory, "
	                                                                        "interrupting the currently displayed one.",
	                                                                        parent);

	loop_display_property_ = new rviz_common::properties::BoolProperty(
	    "Loop Animation", false, "Indicates whether the last received path is to be animated in a loop", parent,
	    SLOT(changedLoopDisplay()), this);

	trail_display_property_ = new rviz_common::properties::BoolProperty("Show Trail", false, "Show a path trail", parent,
	                                                                    SLOT(changedTrail()), this);

	state_display_time_property_ =
	    new rviz_common::properties::EditableEnumProperty("State Display Time", "0.05 s",
	                                                      "The amount of wall-time to wait in between displaying "
	                                                      "states along a received trajectory path",
	                                                      parent);
	state_display_time_property_->addOptionStd("REALTIME");
	state_display_time_property_->addOptionStd("0.05 s");
	state_display_time_property_->addOptionStd("0.1 s");
	state_display_time_property_->addOptionStd("0.5 s");

	trail_step_size_property_ = new rviz_common::properties::IntProperty(
	    "Trail Step Size", 1, "Specifies the step size of the samples shown in the trajectory trail.", parent,
	    SLOT(changedTrail()), this);
	trail_step_size_property_->setMin(1);

	// robot properties
	robot_property_ = new rviz_common::properties::Property("Robot", QString(), QString(), parent);
	robot_visual_enabled_property_ =
	    new rviz_common::properties::BoolProperty("Show Robot Visual", true,
	                                              "Indicates whether the geometry of the robot as defined for "
	                                              "visualisation purposes should be displayed",
	                                              robot_property_, SLOT(changedRobotVisualEnabled()), this);

	robot_collision_enabled_property_ =
	    new rviz_common::properties::BoolProperty("Show Robot Collision", false,
	                                              "Indicates whether the geometry of the robot as defined "
	                                              "for collision detection purposes should be displayed",
	                                              robot_property_, SLOT(changedRobotCollisionEnabled()), this);

	robot_alpha_property_ =
	    new rviz_common::properties::FloatProperty("Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
	                                               robot_property_, SLOT(changedRobotAlpha()), this);
	robot_alpha_property_->setMin(0.0);
	robot_alpha_property_->setMax(1.0);

	robot_color_property_ = new rviz_common::properties::ColorProperty("Fixed Robot Color", QColor(150, 50, 150),
	                                                                   "The color of the animated robot",
	                                                                   robot_property_, SLOT(changedRobotColor()), this);

	enable_robot_color_property_ =
	    new rviz_common::properties::BoolProperty("Use Fixed Robot Color", false,
	                                              "Specifies whether the fixed robot color should be used."
	                                              " If not, the original color is used.",
	                                              robot_property_, SLOT(enabledRobotColor()), this);

	// planning scene properties
	scene_enabled_property_ = new rviz_common::properties::BoolProperty("Scene", true, "Show Planning Scene", parent,
	                                                                    SLOT(changedSceneEnabled()), this);

	scene_alpha_property_ =
	    new rviz_common::properties::FloatProperty("Scene Alpha", 0.9f, "Specifies the alpha for the scene geometry",
	                                               scene_enabled_property_, SLOT(renderCurrentScene()), this);
	scene_alpha_property_->setMin(0.0);
	scene_alpha_property_->setMax(1.0);

	scene_color_property_ = new rviz_common::properties::ColorProperty(
	    "Scene Color", QColor(50, 230, 50), "The color for the planning scene obstacles (if a color is not defined)",
	    scene_enabled_property_, SLOT(renderCurrentScene()), this);

	attached_body_color_property_ = new rviz_common::properties::ColorProperty(
	    "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies", scene_enabled_property_,
	    SLOT(changedAttachedBodyColor()), this);

	octree_render_property_ =
	    new rviz_common::properties::EnumProperty("Voxel Rendering", "Occupied Voxels", "Select voxel type.",
	                                              scene_enabled_property_, SLOT(renderCurrentScene()), this);

	octree_render_property_->addOption("Occupied Voxels", OCTOMAP_OCCUPIED_VOXELS);
	octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
	octree_render_property_->addOption("All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

	octree_coloring_property_ =
	    new rviz_common::properties::EnumProperty("Voxel Coloring", "Z-Axis", "Select voxel coloring mode",
	                                              scene_enabled_property_, SLOT(renderCurrentScene()), this);

	octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
	octree_coloring_property_->addOption("Cell Probability", OCTOMAP_PROBABLILTY_COLOR);

	marker_visual_ = new MarkerVisualizationProperty("Markers", parent);
	connect(marker_visual_, SIGNAL(allAtOnceChanged(bool)), this, SLOT(onAllAtOnceChanged(bool)));
}

TaskSolutionVisualization::~TaskSolutionVisualization() {
	clearTrail();
	next_solution_to_display_.reset();
	displaying_solution_.reset();

	scene_render_.reset();
	robot_render_.reset();
	delete slider_dock_panel_;
	delete marker_visual_;

	if (main_scene_node_)
		main_scene_node_->getCreator()->destroySceneNode(main_scene_node_);
}

void TaskSolutionVisualization::onInitialize(Ogre::SceneNode* scene_node, rviz_common::DisplayContext* context) {
	// Save pointers for later use
	parent_scene_node_ = scene_node;
	context_ = context;
	main_scene_node_ = parent_scene_node_->getCreator()->createSceneNode();
	trail_scene_node_ = parent_scene_node_->getCreator()->createSceneNode();

	// Load trajectory robot
	robot_render_.reset(new RobotStateVisualization(main_scene_node_, context_, "Solution Trajectory", robot_property_));
	robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
	robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
	changedRobotAlpha();
	enabledRobotColor();
	robot_render_->setVisible(true);

	scene_render_.reset(new PlanningSceneRender(main_scene_node_, context_, RobotStateVisualizationPtr()));
	scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

	marker_visual_->onInitialize(main_scene_node_, context_);

	rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
	if (window_context) {
		slider_panel_ = new TaskSolutionPanel(window_context->getParentWindow());
		slider_dock_panel_ = window_context->addPane(display_->getName() + " - Slider", slider_panel_);
		slider_dock_panel_->setIcon(display_->getIcon());
		connect(slider_dock_panel_, SIGNAL(visibilityChanged(bool)), this, SLOT(sliderPanelVisibilityChange(bool)));
		slider_panel_->onInitialize();
	}
}

void TaskSolutionVisualization::setName(const QString& name) {
	if (slider_dock_panel_)
		slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TaskSolutionVisualization::onRobotModelLoaded(const moveit::core::RobotModelConstPtr& robot_model) {
	// Error check
	if (!robot_model) {
		RCLCPP_ERROR(LOGGER, "No robot model found");
		return;
	}

	scene_.reset(new planning_scene::PlanningScene(robot_model));

	robot_render_->load(*robot_model->getURDF());  // load rviz robot
	enabledRobotColor();  // force-refresh to account for saved display configuration
}

void TaskSolutionVisualization::reset() {
	clearTrail();
	next_solution_to_display_.reset();
	displaying_solution_.reset();
	current_state_ = -1;
	if (slider_panel_)
		slider_panel_->update(-1);

	robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
	robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
	scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

	if (main_scene_node_->getParent())
		parent_scene_node_->removeChild(main_scene_node_);
}

void TaskSolutionVisualization::clearTrail() {
	qDeleteAll(trail_);
	trail_.clear();
}

void TaskSolutionVisualization::changedLoopDisplay() {
	// restart animation if current_state_ is at end and looping got activated
	if (displaying_solution_ && loop_display_property_->getBool() && slider_panel_ && slider_panel_->isVisible() &&
	    current_state_ + 1 >= static_cast<int>(displaying_solution_->getWayPointCount())) {
		current_state_ = -1;
		slider_panel_->pauseButton(false);
	}
}

void TaskSolutionVisualization::changedTrail() {
	clearTrail();
	DisplaySolutionPtr t = displaying_solution_;

	if (!t || !trail_display_property_->getBool()) {
		setVisibility(trail_scene_node_, main_scene_node_, false);
		return;
	}

	setVisibility(main_scene_node_, parent_scene_node_, true);
	setVisibility(trail_scene_node_, main_scene_node_, true);

	int stepsize = trail_step_size_property_->getInt();
	trail_.resize(t->getWayPointCount() / stepsize);
	for (std::size_t i = 0; i < trail_.size(); i++) {
		int waypoint_i = std::min(i * stepsize, t->getWayPointCount() - 1);  // limit to last trajectory point
		rviz_default_plugins::robot::Robot* r = new rviz_default_plugins::robot::Robot(
		    trail_scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), nullptr);
		r->load(*scene_->getRobotModel()->getURDF());
		r->setVisualVisible(robot_visual_enabled_property_->getBool());
		r->setCollisionVisible(robot_collision_enabled_property_->getBool());
		r->setAlpha(robot_alpha_property_->getFloat());
		r->update(PlanningLinkUpdater(t->getWayPointPtr(waypoint_i)));
		if (enable_robot_color_property_->getBool())
			setRobotColor(r, robot_color_property_->getColor());
		r->setVisible(waypoint_i <= current_state_);
		trail_[i] = r;
	}
}

void TaskSolutionVisualization::changedRobotAlpha() {
	robot_render_->setAlpha(robot_alpha_property_->getFloat());
	for (auto& waypoint : trail_)
		waypoint->setAlpha(robot_alpha_property_->getFloat());
}

void TaskSolutionVisualization::changedRobotVisualEnabled() {
	robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
	renderCurrentWayPoint();
	for (auto& waypoint : trail_)
		waypoint->setVisualVisible(robot_visual_enabled_property_->getBool());
}

void TaskSolutionVisualization::changedRobotCollisionEnabled() {
	robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
	renderCurrentWayPoint();
	for (auto& waypoint : trail_)
		waypoint->setCollisionVisible(robot_collision_enabled_property_->getBool());
}

void TaskSolutionVisualization::onEnable() {
	if (slider_panel_ && slider_panel_was_visible_)
		slider_panel_->onEnable();
}

void TaskSolutionVisualization::onDisable() {
	// make all scene nodes invisible
	if (main_scene_node_->getParent())
		parent_scene_node_->removeChild(main_scene_node_);

	displaying_solution_.reset();
	next_solution_to_display_.reset();
	current_state_ = -1;
	if (slider_panel_) {
		slider_panel_was_visible_ = slider_panel_->isVisible();
		slider_panel_->onDisable();
	}
}

void TaskSolutionVisualization::interruptCurrentDisplay() {
	if (!locked_)
		drop_displaying_solution_ = true;
}

void TaskSolutionVisualization::onAllAtOnceChanged(bool all_at_once) {
	if (!displaying_solution_)
		return;
	clearMarkers();

	if (all_at_once)
		addMarkers(displaying_solution_);
	else if (current_state_ >= 0)
		renderCurrentWayPoint();
}

float TaskSolutionVisualization::getStateDisplayTime() {
	std::string tm = state_display_time_property_->getStdString();
	if (tm == "REALTIME")
		return -1.0;
	else {
		boost::replace_all(tm, "s", "");
		boost::trim(tm);
		float t = 0.05f;
		try {
			t = boost::lexical_cast<float>(tm);
		} catch (const boost::bad_lexical_cast& ex) {
			state_display_time_property_->setStdString("0.05 s");
		}
		return t;
	}
}

void TaskSolutionVisualization::update(float wall_dt, float /*ros_dt*/) {
	if (drop_displaying_solution_) {
		current_state_ = -1;
		displaying_solution_.reset();
		if (slider_panel_)
			slider_panel_->update(-1);
		drop_displaying_solution_ = false;
	}
	if (current_state_ < 0 ||  // last animation finished
	    (slider_panel_ && slider_panel_->isPaused())) {
		boost::mutex::scoped_lock lock(display_solution_mutex_);

		// new trajectory available to display?
		if (next_solution_to_display_ && (!locked_ || !displaying_solution_)) {
			current_state_ = -1;
			animating_ = true;
			displaying_solution_ = next_solution_to_display_;
			changedTrail();
			if (slider_panel_)
				slider_panel_->update(next_solution_to_display_->getWayPointCount());
		}
		// also reset if locked_
		next_solution_to_display_.reset();
	}
	if (!displaying_solution_) {
		animating_ = false;
		setVisibility();
		return;
	}

	int previous_state = current_state_;
	// for an empty trajectory, show the start and end state at least
	int waypoint_count = displaying_solution_->getWayPointCount();
	int max_state_index = std::max<int>(1, waypoint_count);
	if (slider_panel_ && slider_panel_->isVisible()) {
		animating_ = !slider_panel_->isPaused();
		if (current_state_ >= 0)
			// user can override current_state_ at any time with slider
			current_state_ = slider_panel_->getSliderPosition();
	} else if (current_state_ < max_state_index)
		animating_ = true;  // auto-activate animation if slider_panel_ is hidden

	if (animating_ && current_state_ == previous_state) {
		// auto-advance current_state_ based on time progress
		current_state_time_ += wall_dt;

		float tm = getStateDisplayTime();
		if (current_state_ < 0) {  // special case indicating restart of animation
			current_state_ = 0;
			current_state_time_ = 0.0;
			trail_scene_node_->setVisible(false);
		} else if (tm < 0.0) {  // using realtime: skip to next waypoint based on elapsed display time
			while (current_state_ < max_state_index && (tm = displaying_solution_->getWayPointDurationFromPrevious(
			                                                current_state_ + 1)) < current_state_time_) {
				++current_state_;
				current_state_time_ -= tm;
			}
		} else if (current_state_time_ > tm) {  // fixed display time per state
			++current_state_;
			current_state_time_ = 0.0;
		}
	} else if (current_state_ != previous_state) {  // current_state_ changed from slider
		current_state_time_ = 0.0;
	}

	if ((waypoint_count > 0 && current_state_ >= max_state_index) ||
	    (waypoint_count == 0 && current_state_ > max_state_index)) {  // animation finished?
		if (loop_display_property_->getBool())
			current_state_ = -1;  // restart in next cycle
		else {
			current_state_ = max_state_index;
			if (slider_panel_)
				slider_panel_->pauseButton(true);
		}
		setVisibility();
		return;
	}
	if (current_state_ == previous_state)
		return;

	renderWayPoint(current_state_, previous_state);

	// show / hide trail between start .. end
	int stepsize = trail_step_size_property_->getInt();
	int start = std::max(0, previous_state / stepsize);
	int end = current_state_ / stepsize;
	bool show = start <= end;
	if (!show)
		std::swap(start, end);
	end = std::min<int>(end, trail_.size());
	for (; start < end; ++start)
		trail_[start]->setVisible(show);

	setVisibility();
}

void TaskSolutionVisualization::renderCurrentWayPoint() {
	if (displaying_solution_)
		renderWayPoint(current_state_, -1);
}

void TaskSolutionVisualization::renderWayPoint(size_t index, int previous_index) {
	size_t waypoint_count = displaying_solution_->getWayPointCount();
	moveit::core::RobotStateConstPtr robot_state;
	planning_scene::PlanningSceneConstPtr scene;
	if (index + 1 >= waypoint_count) {
		if (index == 0 && waypoint_count == 0)
			// special case: render start scene
			scene = displaying_solution_->startScene();
		else  // render end state
			scene = displaying_solution_->scene(waypoint_count);
		renderPlanningScene(scene);
		robot_state.reset(new moveit::core::RobotState(scene->getCurrentState()));
	} else {
		auto idx_pair = displaying_solution_->indexPair(index);
		scene = displaying_solution_->scene(idx_pair);

		if (previous_index < 0 || previous_index >= static_cast<int>(waypoint_count) ||
		    displaying_solution_->indexPair(previous_index).first != idx_pair.first) {
			// switch to new stage: show new planning scene
			renderPlanningScene(scene);
			// switch to markers of next sub trajectory?
			if (!marker_visual_->allAtOnce()) {
				marker_visual_->clearMarkers();
				marker_visual_->addMarkers(displaying_solution_->markersOfSubTrajectory(idx_pair.first));
			}
			Q_EMIT activeStageChanged(displaying_solution_->creatorId(idx_pair));
		}
		robot_state = displaying_solution_->getWayPointPtr(idx_pair);
	}

	QColor attached_color = attached_body_color_property_->getColor();
	std_msgs::msg::ColorRGBA color;
	color.r = attached_color.redF();
	color.g = attached_color.greenF();
	color.b = attached_color.blueF();
	color.a = 1.0f;

	planning_scene::ObjectColorMap color_map;
	scene->getKnownObjectColors(color_map);
	robot_render_->update(robot_state, color, color_map);
	marker_visual_->update(*scene, *robot_state);

	if (slider_panel_)
		slider_panel_->setSliderPosition(index);
}

void TaskSolutionVisualization::renderPlanningScene(const planning_scene::PlanningSceneConstPtr& scene) {
	if (!scene_render_ || !scene_enabled_property_->getBool())
		return;

	QColor color = scene_color_property_->getColor();
	Ogre::ColourValue env_color(color.redF(), color.greenF(), color.blueF());
	color = attached_body_color_property_->getColor();
	Ogre::ColourValue attached_color(color.redF(), color.greenF(), color.blueF());

	scene_render_->renderPlanningScene(
	    scene, env_color, attached_color, static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt()),
	    static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt()), scene_alpha_property_->getFloat());
}

void TaskSolutionVisualization::showTrajectory(const moveit_task_constructor_msgs::msg::Solution& msg) {
	DisplaySolutionPtr s(new DisplaySolution);
	s->setFromMessage(scene_, msg);
	showTrajectory(s, false);
}

void TaskSolutionVisualization::showTrajectory(const DisplaySolutionPtr& s, bool lock_display) {
	if (lock_display || !s->empty()) {
		boost::mutex::scoped_lock lock(display_solution_mutex_);
		next_solution_to_display_ = s;
		if (lock_display)
			locked_ = drop_displaying_solution_ = true;
		else if (interrupt_display_property_->getBool())
			interruptCurrentDisplay();
	}
}

void TaskSolutionVisualization::unlock() {
	locked_ = false;
}

void TaskSolutionVisualization::clearMarkers() {
	marker_visual_->clearMarkers();
}

void TaskSolutionVisualization::addMarkers(const moveit_rviz_plugin::DisplaySolutionPtr& s) {
	if (!s || (!marker_visual_->allAtOnce() && s->numSubSolutions() > 1))
		return;

	for (size_t i = 0, end = s->numSubSolutions(); i != end; ++i) {
		marker_visual_->addMarkers(s->markersOfSubTrajectory(i));
	}
}

void TaskSolutionVisualization::changedRobotColor() {
	if (enable_robot_color_property_->getBool())
		setRobotColor(&(robot_render_->getRobot()), robot_color_property_->getColor());
}

void TaskSolutionVisualization::enabledRobotColor() {
	if (enable_robot_color_property_->getBool())
		setRobotColor(&(robot_render_->getRobot()), robot_color_property_->getColor());
	else
		unsetRobotColor(&(robot_render_->getRobot()));
}

void TaskSolutionVisualization::changedAttachedBodyColor() {
	renderCurrentWayPoint();
}

void TaskSolutionVisualization::unsetRobotColor(rviz_default_plugins::robot::Robot* robot) {
	for (auto& link : robot->getLinks())
		link.second->unsetColor();
}

void TaskSolutionVisualization::setRobotColor(rviz_default_plugins::robot::Robot* robot, const QColor& color) {
	for (auto& link : robot->getLinks())
		link.second->setColor(color.redF(), color.greenF(), color.blueF());
}

void TaskSolutionVisualization::sliderPanelVisibilityChange(bool enable) {
	if (!slider_panel_)
		return;

	if (enable) {
		// also enable display
		display_->setEnabled(true);
		slider_panel_->onEnable();
	} else
		slider_panel_->onDisable();

	setVisibility();
}

void TaskSolutionVisualization::changedSceneEnabled() {
	if (!scene_render_)
		return;
	setVisibility(scene_render_->getGeometryNode(), main_scene_node_, scene_enabled_property_->getBool());
}

void TaskSolutionVisualization::renderCurrentScene() {
	if (scene_render_ && scene_enabled_property_->getBool() && current_state_ >= 0)
		renderPlanningScene(displaying_solution_->scene(current_state_));
}

void TaskSolutionVisualization::setVisibility(Ogre::SceneNode* node, Ogre::SceneNode* parent, bool visible) {
	if (node != main_scene_node_ && !main_scene_node_->getParent())
		return;  // main scene node is detached

	if (visible && node->getParent() != parent) {
		parent->addChild(node);
		// if main scene node became attached, also update visibility of other nodes
		if (node == main_scene_node_) {
			if (scene_render_)
				setVisibility(scene_render_->getGeometryNode(), main_scene_node_, scene_enabled_property_->getBool());
			setVisibility(trail_scene_node_, main_scene_node_, loop_display_property_->getBool());
		}
	} else if (!visible && node->getParent())
		node->getParent()->removeChild(node);
}

void TaskSolutionVisualization::setVisibility() {
	// main scene node is visible if animation, trail, or panel is shown, or if display is locked
	setVisibility(
	    main_scene_node_, parent_scene_node_,
	    display_->isEnabled() && displaying_solution_ &&
	        (animating_ || locked_ || trail_scene_node_->getParent() || (slider_panel_ && slider_panel_->isVisible())));
}

}  // namespace moveit_rviz_plugin
