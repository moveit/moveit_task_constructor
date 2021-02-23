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
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_task_constructor_demo {
constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}

void PickPlaceTask::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
	ros::NodeHandle pnh("~");

	// Planning group properties
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", hand_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
	support_surfaces_ = { surface_link_ };

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

// The required sequences for a successful pick-and-place task are defined below. The entire process is comprised of six
// "high level" movements: Checking if the object to be picked is already attached to the robot arm, opening the robot's
// hand, moving the robot to the pre-grasp position, grasping the object, moving the robot to a pre-place position,
// placing the object, and finally, moving the robot to a standstill position. These six sequences are listed on the
// least indented column of the "Motion Planning Tasks" widget in Rviz. These sequences are not planned or executed but
// defined as part of moveit's task constructor object below. The stages are be described in more detail below.

void PickPlaceTask::init() {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	const std::string object = object_name_;

	// Reset ROS introspection before constructing the new object
	// TODO(henningkayser): verify this is a bug, fix if possible
    // The structure of the pick-and-place process is collected inside a moveit::task_constructor::Task.
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

    //The robot movement is planned by different planners: a cartesian planner, a pipeline planner, or a joint
    //interpolation planner.
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	t.setProperty("group", arm_group_name_);
	t.setProperty("eef", eef_name_);
	t.setProperty("hand", hand_group_name_);
	t.setProperty("hand_grasping_frame", hand_frame_);
	t.setProperty("ik_frame", hand_frame_);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
    // Before beginning with the operation, one needs to verify if the object is not already attached to the robot. 
	Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});

		current_state_ptr = applicability_filter.get();
		t.add(std::move(applicability_filter));
	}

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(hand_group_name_);
		stage->setGoal(hand_open_pose_);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
    // This is the second major phase of picking the object. This phase moves to the start of the picking location. The
    // movement is generated by a stage::Connect object which itself does not declare any movements but acts as a bridge
    // between two stages that are precisely defined.  
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
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
        // The third major stage of the pick-and-place sequence is defined: picking the object. The grasp is defined
        // inside a SerialContainer. A SerialContainer consists of a sequence of subordinate tasks. Stages other than
        // these subordinate tasks can communicate only with the entire SerialContainer and do not need to know each
        // subordinate task.  
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
		{
            // This movement approaches the object and stops right before it through a MoveRelative stage. The stage
            // resembles the Connect because it does not define an end or starting point but move across space as defined
            // as a geometryMsgs. In contrast to the Connect stage, however, the planner does not have any flexibility in
            // the cartesian structure of the movement as this is precisely defined.
            auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", hand_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
            // The object grasp is defined by combining two stages, a GenerateGraspPose and a ComputeIK pose. The
            // generateGraspPose latches several possible target poses onto the object to be grasped.  The
            // GenerateGraspPose circles in AngleDelta increments along the object creating one target pose in each
            // instance.  
            auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose(hand_open_pose_);
            stage->setObject(object);
            stage->setAngleDelta(M_PI / 4);
            stage->setMonitoredStage(current_state_ptr);  // Hook into current state

            // The ComputeIK stage defines the grasp pose for the end-effector. The ComputeIK stage attempts to compute
            // the appropriate joint parameters from two components: The target posed sampled from the GenerateGraspPose
            // stage and the user-defined grasp_frame_transform:  The proposed solution is post-multiplied by the
            // inverse of the grasp_frame_transform to yield a target frame to be attained by the robot's end-effector.
            // If an IK solution is found, the end effectors frame times the rasp_frame_transform equals the frame
            // generated in the GenerateGraspPose stage and the object can be picked.
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
        {
            // After the grasp frame has been determined, the next step is to pick up the object. At first, the planning
            // scene needs to be modified to allow the robot to pick up the object. This stage does not alter the
            // robot's position.
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

		/****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
		{
            // To close the hand, the robot moves from the position of an open hand to a position with a closed hand.
            // Similar to moving from the initial pose to the pose with `open hand` as done in the beginning, the MoveTo
            // stage is used. 
            auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
			stage->setGroup(hand_group_name_);
			stage->setGoal(hand_close_pose_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
        // The next four stages modify the planning scene again by attaching the object to the hand, and lifting the
        // object while guaranteeing that the cylinder does not touch the ground.
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
			vec.header.frame_id = world_frame_;
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
        // Similar to the `move to pick` stage, the `move to place` stage is defined implicitly as a Connect stage: No
        // robot movement on its own is defined but the movement bridges the previous `pick object` and the following
        // `place object` serial containers.  
        auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
    // Placing the object constitutes again a sequence of operations that are combined into a SerialContainer so that
    // outside stages only need to communicate with the boundaries of the SerialContainer. The process is very similar
    // to the pick object process described above and thus not illustrated.  
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
			vec.header.frame_id = world_frame_;
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

			// Set target pose
			geometry_msgs::PoseStamped p;
			p.header.frame_id = object_reference_frame_;
			p.pose = place_pose_;
			p.pose.position.z += 0.5 * object_dimensions_[0] + place_surface_offset_;
			stage->setPose(p);
			stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(2);
			wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *          Open Hand                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
			stage->setGroup(hand_group_name_);
			stage->setGoal(hand_open_pose_);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (hand, object)        *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(
			    object_name_,
			    t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
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
		stage->setGoal(arm_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	ros::NodeHandle pnh("~");
	int max_solutions = pnh.param<int>("max_solutions", 10);

	try {
		task_->plan(max_solutions);
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

bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_->solutions().front()->fillMessage(execute_goal.solution);
	execute_.sendGoal(execute_goal);
	execute_.waitForResult();
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}

	return true;
}
}  // namespace moveit_task_constructor_demo
