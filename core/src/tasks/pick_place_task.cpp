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

#include <moveit/task_constructor/tasks/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit {
namespace task_constructor {
namespace tasks {

constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name)
  : task_name_(task_name) {}

void PickPlaceTask::init(const Parameters& parameters)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  task_.reset();
  task_.reset(new moveit::task_constructor::Task(task_name_));
  Task& t = *task_;
  t.loadRobotModel();

  // Sampling planner
  // TODO(henningkayser): Setup and parameterize alternative planners
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", parameters.arm_group_name_);
  t.setProperty("eef", parameters.eef_name_);
  t.setProperty("hand", parameters.hand_group_name_);
  t.setProperty("ik_frame", parameters.hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  Stage* current_state = nullptr;  // Forward current_state on to grasp pose generator
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(10);

    // Verify that object is not attachd
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase& s, std::string& comment) {
      s.start()->scene()->printKnownObjects(std::cout);
      if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
      {
        comment = "object with id '" + parameters.object_name_ + "' is already attached and cannot be picked";
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
    stage->setGroup(parameters.hand_group_name_);
    stage->setGoal(parameters.hand_open_pose_);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {  // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
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
      stage->properties().set("link", parameters.hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(parameters.approach_object_min_dist_, parameters.approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = parameters.object_name_;
      vec.vector.z = -1.0;
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
      stage->setPreGraspPose(parameters.hand_open_pose_);
      stage->setObject(parameters.object_name_);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state);  // Hook into current state

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
      //wrapper->setIgnoreCollisions(true);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
  ---- *               Allow Collision (hand object)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          parameters.object_name_,
          t.getRobotModel()->getJointModelGroup(parameters.hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, parameters.hand_group_name_);
      stage->setGoal(parameters.hand_close_pose_);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(parameters.object_name_, parameters.hand_frame_);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ parameters.object_name_ }, "source_container", true);  // TODO(henningkayser): parameterize
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Lift object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(parameters.lift_object_min_dist_, parameters.lift_object_max_dist_);
      stage->setIKFrame(parameters.hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = parameters.world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ parameters.object_name_ }, "source_container", false);  // TODO(henningkayser): parameterize
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
        "move to place", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
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
      stage->properties().set("link", parameters.hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(parameters.place_object_min_dist_, parameters.place_object_max_dist_);

      // Set downward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = parameters.world_frame_;
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
      stage->setObject(parameters.object_name_);
      stage->setPose(parameters.place_pose_);
      // Hook into attach_object_stage which allows us to use the attached object as IK frame
      stage->setMonitoredStage(attach_object_stage);

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
      // TODO(henningkayser): Enable collisions
      wrapper->setIgnoreCollisions(true);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
  ---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Forbid collision (hand, object)        *
     *****************************************************/
    {
      // TODO(henningkayser): Forbid collision after retreat?
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          parameters.object_name_,
          t.getRobotModel()->getJointModelGroup(parameters.hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(parameters.object_name_, parameters.hand_frame_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Retreat Motion                            *
     *****************************************************/
    {
      // TODO(henningkayser): Do we need this if items are dropped?
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(parameters.retreat_object_min_dist_, parameters.retreat_object_max_dist_);
      stage->setIKFrame(parameters.hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = parameters.hand_frame_;
      vec.vector.z = -1.0;
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
    stage->setGoal(parameters.arm_home_pose_);
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	try {
		task_->plan(10);  // TODO: parameterize
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}

}  // namespace tasks
}  // namespace task_constructor
}  // namespace moveit
