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
/* Description:  Get TCP pose, executing time, errors...*/

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/GetMotionSequence.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Position of object robot decide to pick
geometry_msgs::Pose object_pose;
std::vector<double> robot_joint_values;

// 100 Hz
void ObjectCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
  object_pose.position = msg->position;
  object_pose.orientation = msg->orientation;
  // ROS_INFO("Point is updated %f", msg->position.y);
}

// 100 Hz
void JointStatesCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
  robot_joint_values = msg->position;
  ROS_INFO("JP: %f, %f, %f, %f, %f, %f", 
    robot_joint_values[0],
    robot_joint_values[1],
    robot_joint_values[2],
    robot_joint_values[3],
    robot_joint_values[4],
    robot_joint_values[5]);
}

int main(int argc, char** argv)
{
  // ROS setup
  ros::init(argc, argv, "irb1200_robot_monitoring");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe object pose
  ros::Subscriber object_pose_subscriber = node_handle.subscribe("object_pose", 10, ObjectCallBack);
  ros::Subscriber joint_states_subcriber = node_handle.subscribe("joint_states", 10, JointStatesCallBack);

  // Define planning group, base frame and end effector link
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string BASE_FRAME = "base_link";
  static const std::string EE_BASE_LINK = "link_6"; // The link end-effector will be attached on
  static const std::string TOOL_LINK = "tool0"; // Working point

  // Load the robot description on ROS parameter server
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  // Construct a robot model, which contains the robot's kinematic info
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  // Construct a planning scene from the robot model loader
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  // Get robot state
  moveit::core::RobotStatePtr robot_state(
                  new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  // 
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  
  robot_joint_values = {0, 0, 0, 0, 0, 0};
  
  geometry_msgs::Pose robot_current_pose;

  double sampling_time;
  double sampling_rate;
  double tracking_error;
  double dy = 0;
  double vy = 0;
  double current_pose = 0;

  Eigen::Isometry3d tool0_state;

  while (ros::ok())
  {

    // sampling_time = ros::Time::now().toSec();

    // Option 1: Get TCP pose using move group interface
    // robot_current_pose = move_group_interface.getCurrentPose().pose;
    // ROS_INFO("Tool positions: %f, %f, %f", 
    // robot_current_pose.position.x,
    // robot_current_pose.position.y,
    // robot_current_pose.position.z);

    // Option 2: Get TCP pose using robot state
    // robot_state = move_group_interface.getCurrentState();
    // const Eigen::Isometry3d &tool0_state = robot_state->getGlobalLinkTransform(TOOL_LINK);
    // tool0_state = robot_state->getGlobalLinkTransform(TOOL_LINK);
    // robot_current_pose = tf2::toMsg(tool0_state);
    // dy = tool0_state.translation().y() - current_pose;
    // ROS_INFO("Tool positions: %f, %f, %f", 
    // robot_current_pose.position.x,
    // robot_current_pose.position.y,
    // robot_current_pose.position.z);
    // current_pose = tool0_state.translation().y();
    // sampling_time = ros::Time::now().toSec() - sampling_time;
    // vy = dy/sampling_time;
    // sampling_rate = 1/sampling_time;
    // tracking_error = tool0_state.translation().y() - object_pose.position.y;
  
    // Print data
    // ROS_INFO("Tracking error: %f. Sampling rate: %f", tracking_error, sampling_rate);
    // ROS_INFO("y = %f. Sampling time = %f", current_pose, sampling_time);
    // ROS_INFO("Catesian vel of y: %f", vy);

    ros::Duration(0.1).sleep();
  }

  return 0;
}