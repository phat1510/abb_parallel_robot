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
/* Demo 1: Pose planning, Joint-space planning, Cartesian planning */
/* Plugin: OMPL*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  // Create a node
  ros::init(argc, argv, "irb1200_demo_01");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string BASE_FRAME = "base_link";
  static const std::string EE_BASE_LINK = "link_6";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  // Set planner id
  move_group_interface.setPlannerId("PTP");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  visual_tools.publishText(text_pose, "IRB1200 DEMO 01", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Display base link coordinate
  geometry_msgs::Pose base_link;
  base_link.orientation.w = 1;
  base_link.position.x = 0;
  base_link.position.y = 0;
  base_link.position.z = 0;
  visual_tools.publishAxis(base_link,0.25);
  visual_tools.trigger();

  // Start the demo
  ROS_INFO_NAMED("tutorial", "Setup finished. Ready to start...");
  visual_tools.prompt("Press 'Next' to go to home position...");

  while(ros::ok())
  {

    /*---------------------------HOMING - POSE PLANNING--------------------*/
    // Set target pose
    geometry_msgs::Pose home_pose;
    home_pose.orientation.x = 0;
    home_pose.orientation.y = sin(tau/8);
    home_pose.orientation.z = 0;
    home_pose.orientation.w = cos(tau/8);
    home_pose.position.x = 0.4;
    home_pose.position.y = 0;
    home_pose.position.z = 0.3;

    // Set pose target to the planner
    move_group_interface.setPoseTarget(home_pose);

    // Ask the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // 
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal planning %s", success ? "" : "FAILED");
    move_group_interface.execute(my_plan);

    // Visualizing plans
    visual_tools.publishAxisLabeled(home_pose, "Home");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XXLARGE);
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
    visual_tools.trigger();

    /*---------------------------JOINT-SPACE PLANNING--------------------*/
    // Rotate 90 degrees

    visual_tools.prompt("Press 'Next' to plan and execute in joint-space...");
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -tau / 4;
    move_group_interface.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Joint-space goal planning %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    //visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
    visual_tools.trigger();
    
    // Execute the computed plan
    move_group_interface.execute(my_plan);


    /*---------------------------CARTESIAN PLANNING - DISABLE TEMPORARILY---------------*/
    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
    // Pull requests are welcome.
    // ^^^^^^^^^^^^
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    
    // Wait for Cartesian planning
    visual_tools.prompt("Press 'Next' to plan Cartesian path...");
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    geometry_msgs::Pose eef_current_pose = move_group_interface.getCurrentPose(EE_BASE_LINK).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(eef_current_pose);

    
    geometry_msgs::Pose target_pose2 = eef_current_pose;
    target_pose2.position.z += 0.12;// up
    waypoints.push_back(target_pose2);  
    target_pose2.position.y -= 0.2;// left
    waypoints.push_back(target_pose2);  
    target_pose2.position.z -= 0.2;   // down
    waypoints.push_back(target_pose2);  
    target_pose2.position.x +=0.2;
    target_pose2.position.y +=0.3;
    target_pose2.position.z +=0.2;
    waypoints.push_back(target_pose2);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Cartesian path (%.2f%% acheived)", fraction * 100.0);


    // Visualize the plan in RViz
    //visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::YELLOW, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    
    // Execute the computed plan
    visual_tools.prompt("Press 'next' to execute");
    move_group_interface.execute(trajectory);
    visual_tools.prompt("Press 'next' to home planning");
    //-----*/

    /*---------------------------BACK TO HOME--------------------*/
    
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    move_group_interface.setPoseTarget(home_pose);

    // 
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Display homing path... %s", success ? "" : "FAILED");

    // Visualizing plans
    ROS_INFO_NAMED("tutorial", "Display homing path");
    visual_tools.publishAxisLabeled(home_pose, "Home");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XXLARGE);
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel(EE_BASE_LINK),joint_model_group);
    visual_tools.trigger();

    // Execute the computed plan
    visual_tools.prompt("Press 'Next' to execute the plan...");
    move_group_interface.execute(my_plan);

    // 
    visual_tools.prompt("Press 'next' to shutdown ROS");
    visual_tools.deleteAllMarkers();
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    //ros::shutdown();
  }
  
  return 0;
}