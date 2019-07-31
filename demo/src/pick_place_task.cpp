/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

#include <moveit_task_constructor_demo/pick_place_task.h>

namespace moveit_task_constructor_demo
{
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true)
{
  ROS_INFO("waiting for task execution");
  // execute_.waitForServer();

  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  ros::NodeHandle pnh("~");

  // Planning group properties
  group_name_ = pnh.param<std::string>("group_name", "manipulator");
  hand_name_ = pnh.param<std::string>("hand_name", "hand");
  eef_name_ = pnh.param<std::string>("eef_name", "hand");
  hand_frame_ = pnh.param<std::string>("hand_frame", "panda_hand");

  // Object + surface
  table_surface_frame_ = pnh.param<std::string>("table_surface_frame", "table_top");
  object_surface_frame_ = pnh.param<std::string>("object_surface_frame", "table");
  surface_link_ = pnh.param<std::string>("surface_link", "table");
  support_surfaces_ = { surface_link_ };
  //  table_name_ = pnh.param<std::string>("table_name", "FAKE");
  table_height_ = pnh.param<double>("table_height", 0.3);
  table_length_ = pnh.param<double>("table_length", 0.5);
  table_width_ = pnh.param<double>("table_width", 0.5);

  object_name_ = pnh.param<std::string>("object_name", "object");
  object_height_ = pnh.param<double>("object_height", 0.2);
  object_radius_ = pnh.param<double>("object_radius", 0.03);
  // Pick
  approach_object_min_dist_ = pnh.param<double>("approach_object_min_dist", 0.1);
  approach_object_max_dist_ = pnh.param<double>("approach_object_max_dist", 0.15);

  // Lift
  lift_object_min_dist_ = pnh.param<double>("lift_object_min_dist", 0.01);
  lift_object_max_dist_ = pnh.param<double>("lift_object_max_dist", 0.1);

  // Place
  place_surface_offset_ = pnh.param<double>("place_surface_offset", 0.01);
  place_pos_x_ = pnh.param<double>("place_pos_x", 0.1);
  place_pos_y_ = pnh.param<double>("place_pos_y", 0.1);

  // compute hand grasp frame
  double rotation = pnh.param<double>("grasp_rotation_z", 1.0);
  double grasp_offset_x = pnh.param<double>("grasp_offset_x", 0.1);
  double grasp_offset_z = pnh.param<double>("prasp_offset_z", 0.0);
  grasp_frame_transform_ = Eigen::AngleAxisd(M_PI * rotation, Eigen::Vector3d::UnitZ()) *
                           Eigen::Translation3d(grasp_offset_x, 0, grasp_offset_z);
}

void PickPlaceTask::init()
{
  const std::string object = "object";

  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  task_.reset();
  task_.reset(new moveit::task_constructor::Task(task_name_));
  Task& t = *task_;
  t.loadRobotModel();

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  Stage* current_state = nullptr;  // Forward current_state on to grasp pose generator
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");

    // Verify that object is not attachd
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(object))
      {
        comment = "object with id '" + object + "' is already attached and cannot be picked";
        return false;
      }
      return true;
    });

    current_state = applicability_filter.get();
    t.add(std::move(applicability_filter));
  }

  /****************************************************
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  {  // Open Hand
    auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup("hand");
    stage->setGoal("open");
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {  // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{ { group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  {
    auto grasp = std::make_unique<SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    /****************************************************
---- *               Approach Object                    *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state);  // Hook into current state

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      //  wrapper->setIgnoreCollisions(true);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
---- *               Allow Collision (hand object)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          object, t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, hand_frame_);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ object }, support_surfaces_, true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Lift object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ object }, support_surfaces_, false);
      grasp->insert(std::move(stage));
    }

    // Add grasp container to task
    t.add(std::move(grasp));
  }

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to place", stages::Connect::GroupPlannerVector{ { group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
  {
    auto place = std::make_unique<SerialContainer>("place object");
    t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

    /******************************************************
---- *          Lower Object                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(.03, .13);

      // Set downward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object);
      // stage->setAugmentRotations(false);

      // Set target pose
      geometry_msgs::PoseStamped p;
      p.header.frame_id = object_surface_frame_;
      p.pose.orientation.w = 1;
      p.pose.position.x = place_pos_x_;
      p.pose.position.y = place_pos_y_;
      p.pose.position.z = 0.5 * object_height_ + place_surface_offset_;
      stage->setPose(p);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Forbid collision (hand, object)        *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          object_name_, t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), false);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name_, hand_frame_);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(.12, .25);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.x = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    t.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setGoal("ready");
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }
}

bool PickPlaceTask::plan()
{
  try
  {
    task_->plan(10);
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM("Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    ROS_ERROR("Planning failed");
    return false;
  }
  return true;
}

bool PickPlaceTask::execute()
{
  moveit_task_constructor_msgs::Solution solution;
  task_->solutions().front()->fillMessage(solution);

  ROS_INFO_STREAM("last trajectory in solution:\n" << solution.sub_trajectory.back().trajectory);

  moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  execute_goal.solution = solution;
  execute_.sendGoal(execute_goal);
  execute_.waitForResult();
  moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM("task execution failed and returned: " << execute_.getState().toString());
    return false;
  }

  return true;
}
}
