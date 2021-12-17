/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Phat Do */
/* Description:  */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/MotionSequenceRequest.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//
namespace rvt = rviz_visual_tools;


int main(int argc, char** argv)
{
  // ROS setup
  // ---------
  // Create the node
  ros::init(argc, argv, "IRB1200_PTP_LIN_CIRC");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup planning model
  // --------------------
  // Define planning group, base frame and end effector link
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string BASE_FRAME = "base_link";
  static const std::string EE_BASE_LINK = "link_6"; // The link end-effector will be attached on

  // Load the robot description on ROS parameter server
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
  
  // Construct a robot model, which contains the robot's kinematic info
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Construct a planning scene from the robot model loader
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


  // Get robot state
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  
  // Create pointer to the planning group
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("manipulator");

  // Setup planning pipeline using planning plugin and request adapter on ROS parameter server
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, 
      "/move_group/planning_pipelines/pilz_industrial_motion_planner/planning_plugin", 
      "/move_group/planning_pipelines/pilz_industrial_motion_planner/request_adapters"));


  // Visualization
  // -------------
  
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  visual_tools.publishText(text_pose, "IRB1200_PTP_LIN_CIRC", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Display base link coordinate
  geometry_msgs::Pose base_link;
  base_link.orientation.w = 1;
  base_link.position.x = 0;
  base_link.position.y = 0;
  base_link.position.z = 0;
  visual_tools.publishAxis(base_link, 0.25);
  visual_tools.trigger();

  // Start the demo
  ROS_INFO_NAMED("tutorial", "Setup finished. Ready to start...");
  visual_tools.prompt("Press 'next' to start.");

  //
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;


  // A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // Home position
  geometry_msgs::PoseStamped home_pose;
  home_pose.header.frame_id = "base_link";
  home_pose.pose.position.x = 0.4;
  home_pose.pose.position.y = -0.4;
  home_pose.pose.position.z = 0.4;
  tf2::Quaternion orientation;
  orientation.setRPY(0, tau/4, 0); // tau = 2*pi;
  home_pose.pose.orientation = tf2::toMsg(orientation);

  // Define name of planning group
  req.group_name = "manipulator";
  req.planner_id = "PTP";
  req.allowed_planning_time = 5;
  req.max_velocity_scaling_factor = 0.2;
  req.max_acceleration_scaling_factor = 1;

  // Set constraints for pose goal
  ROS_INFO_NAMED("tutorial", "Construct goal constraints.");
  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  //visual_tools.prompt("Press 'next' to generate the plan.");

  ROS_INFO_NAMED("tutorial", "Generate path.");
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }

  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  //
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Visualize the trajectory
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  //visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' to execute");

  // Execute the plan
  move_group_interface.execute(response.trajectory);
  visual_tools.prompt("Press 'next' to track");
  //visual_tools.prompt("Press 'next' to plan next point");


  // Pose planning and executing
  //
  ROS_INFO_NAMED("tutorial", "Set the state in planning scene to the final state of the last plan.");
  // First, set the state in the planning scene to the final state of the last plan
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  
  // Set goal pose
  geometry_msgs::PoseStamped next_pose;
  next_pose = home_pose;
  next_pose.pose.position.y += 0.3;
  req.planner_id = "LIN";
  req.max_velocity_scaling_factor = 0.1;
  ROS_INFO_NAMED("tutorial", "Construct goal constraints.");
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", next_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  //
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
  visual_tools.trigger();

  // Execute the plan
  move_group_interface.execute(display_trajectory.trajectory.back());
  visual_tools.prompt("Press 'next' to go to next point");
  /* End*/

  
  // Pose planning and executing
  //
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Set goal pose
  next_pose.pose.position.y += 0.1;
  next_pose.pose.position.z -= 0.1;
  req.max_velocity_scaling_factor = 0.5;
  ROS_INFO_NAMED("tutorial", "Construct goal constraints.");
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", next_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  //
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
  visual_tools.trigger();

  // Execute the plan
  move_group_interface.execute(response.trajectory);
  visual_tools.prompt("Press 'next' to go to next point");
  // End

  // Pose planning and executing
  //
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Set goal pose
  next_pose.pose.position.z += 0.1;
  req.max_velocity_scaling_factor = 0.5;
  ROS_INFO_NAMED("tutorial", "Construct goal constraints.");
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", next_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  //
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
  visual_tools.trigger();

  // Execute the plan
  move_group_interface.execute(response.trajectory);
  visual_tools.prompt("Press 'next' to plan CIRC");
  // End

  // Pose planning and executing - CIRC
  // Set start state to current state
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  
  // Set goal pose
  req.planner_id = "CIRC";
  req.path_constraints.name = "interim";
  //req.path_constraints.name = "center";
  geometry_msgs::Pose cent_pose;
  cent_pose = next_pose.pose;
  cent_pose.position.x +=0.1; 
  cent_pose.position.y +=0.1; 
  moveit_msgs::PositionConstraint pos_constr;
  pos_constr.header.frame_id = "base_link";
  pos_constr.link_name = "link_6";
  pos_constr.weight = 1;
  pos_constr.constraint_region.primitive_poses.push_back(cent_pose);
  req.path_constraints.position_constraints.push_back(pos_constr);
  req.max_velocity_scaling_factor = 0.05;

  ROS_INFO_NAMED("tutorial", "Construct goal constraints.");
  next_pose.pose.position.y += 0.2;
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", next_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  //
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
  visual_tools.trigger();

  // Execute the plan
  move_group_interface.execute(response.trajectory);

  // End
  
  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' to set all joints to zero");

  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, setup a joint space goal
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = { 0, 0, 0, 0, 0, 0};
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  
  req.planner_id = "PTP";
  req.max_velocity_scaling_factor = 0.2;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Visualize the trajectory 
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  // Execute the plan
  move_group_interface.execute(response.trajectory);
  ROS_INFO_NAMED("tutorial", "Complete the demo.");
  visual_tools.prompt("Press 'next' to shutdown ROS");
  ros::shutdown();

  return 0;
}