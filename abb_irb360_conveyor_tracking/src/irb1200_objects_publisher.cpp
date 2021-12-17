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
/*
  Target:
  1. Simulate moving objects on conveyor
  2. Perform conveyor tracking in ROS
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//
namespace rvt = rviz_visual_tools;

// 
const double timer_duration = 0.01; // 100Hz timer
double conveyor_vel; // default = 0.4 m/s

// Define planning group, base frame and end effector link
static const std::string PLANNING_GROUP = "manipulator";
static const std::string BASE_FRAME = "base_link";
static const std::string EE_BASE_LINK = "link_6"; 

// Collision objects dimensions
const double CONVEYOR_WIDTH = 0.55;
const double CONVEYOR_LENGTH = 5.0;
const double CONVEYOR_HEIGHT = 0.815; // default 0.815 m
const double BIN_WIDTH = 0.2;
const double BIN_LENGTH = 0.2;
const double BIN_HEIGHT = 0.3;

//
bool received_obj = 0;
int counter_store = 0;
int counter_track = 0;

//
std::vector<geometry_msgs::Pose> objects_storage;
geometry_msgs::Pose object_pose;

//------------------------------//
// Add some collision objects
//------------------------------//
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  int object_number = 6;
  collision_objects.resize(object_number);

  // Define conveyor
  collision_objects[0].id = "conveyor";
  collision_objects[0].header.frame_id = BASE_FRAME;
  // Define conveyor dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);

  collision_objects[0].primitives[0].dimensions[0] = CONVEYOR_WIDTH; // conveyor width
  collision_objects[0].primitives[0].dimensions[1] = CONVEYOR_LENGTH; // conveyor length
  collision_objects[0].primitives[0].dimensions[2] = CONVEYOR_HEIGHT; // conveyor height
  // Define conveyor pose in the robot frame
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = CONVEYOR_LENGTH/2 - CONVEYOR_LENGTH/2;
  collision_objects[0].primitive_poses[0].position.z = (-CONVEYOR_HEIGHT/2) - (0.025); // 0.025 = robot base  - conveyor height = 0.84 - 0.815
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[0].operation = collision_objects[0].ADD;

  // Define scrap bin 1
  collision_objects[1].id = "scrap_bin1";
  collision_objects[1].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[1].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[1].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.94;
  collision_objects[1].primitive_poses[0].position.y = -0.22;
  collision_objects[1].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[1].operation = collision_objects[1].ADD;

  // Define scrap bin 2
  collision_objects[2].id = "scrap_bin2";
  collision_objects[2].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[2].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[2].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.94;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[2].operation = collision_objects[2].ADD;

  // Define scrap bin 3
  collision_objects[3].id = "scrap_bin3";
  collision_objects[3].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[3].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[3].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.94;
  collision_objects[3].primitive_poses[0].position.y = 0.22;
  collision_objects[3].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[3].operation = collision_objects[3].ADD;

  // Define robot enclosure
  collision_objects[4].id = "left_column";
  collision_objects[4].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 0.04;
  collision_objects[4].primitives[0].dimensions[1] = 0.08;
  collision_objects[4].primitives[0].dimensions[2] = 2.7;
  // Define scrap bin pose in the robot frame
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0.1;
  collision_objects[4].primitive_poses[0].position.y = 0.75;
  collision_objects[4].primitive_poses[0].position.z = 2.7/2 - 0.8;
  collision_objects[4].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[4].operation = collision_objects[4].ADD;

  // Define robot enclosure
  collision_objects[5].id = "right_column";
  collision_objects[5].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[5].primitives.resize(1);
  collision_objects[5].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[5].primitives[0].dimensions.resize(3);
  collision_objects[5].primitives[0].dimensions[0] = 0.04;
  collision_objects[5].primitives[0].dimensions[1] = 0.08;
  collision_objects[5].primitives[0].dimensions[2] = 2.7;
  // Define scrap bin pose in the robot frame
  collision_objects[5].primitive_poses.resize(1);
  collision_objects[5].primitive_poses[0].position.x = 0.1;
  collision_objects[5].primitive_poses[0].position.y = -0.75;
  collision_objects[5].primitive_poses[0].position.z = 2.7/2 - 0.8;
  collision_objects[5].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[5].operation = collision_objects[5].ADD;

  // Add the collision object into the world
  planning_scene_interface.addCollisionObjects(collision_objects);
  ROS_INFO("Added collision objects into the world");
}

//------------------------------------------------------------------//
// 
//------------------------------------------------------------------//
bool CheckPose(geometry_msgs::Pose &pose)
{
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  if ( x < 0.66 && x > 0.33 && y > 0.6 && z < 0.15)
    return true;
  else 
  {
    return false;
  }
}

//------------------------------------------------------------------//
// Update the object position 
//------------------------------------------------------------------//
void UpdatePoint(geometry_msgs::Pose &object_pose)
{
  // Update the object position
  object_pose.position.y -= conveyor_vel * timer_duration;
  // ROS_INFO("Point is updated %f", object_pose.position.y);
}

//------------------------------------------------------------------//
// Update object position by 100Hz timer
//------------------------------------------------------------------//
void TimerCallBack(const ros::TimerEvent& e)
{
  
  for (std::size_t i = 0; i < objects_storage.size(); ++i)
  {
    UpdatePoint(objects_storage[i]);
  }
  // ROS_INFO("Point is updated");
}

//------------------------------------------------------------------//
//
//------------------------------------------------------------------//
void ObjectPosesCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
  object_pose.position = msg->position;
  object_pose.orientation = msg->orientation;
  if (CheckPose(object_pose))
  {
    objects_storage.push_back(object_pose);
    received_obj = 1;
    // ROS_INFO("Received a valid object position");
  }
  else
  {
    ROS_WARN("Object x = %f y = %f z = %f is out of robot workspace. Waiting for next object...", 
      object_pose.position.x, object_pose.position.y, object_pose.position.z);
  }
}

void DoneCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::size_t _size = objects_storage.size();
  if (objects_storage.size() > 0)
  {
    for (std::size_t i = 0; i < _size; ++i)
    {
      if (objects_storage.front().position.y <= 0.6 + 0.01) // default 0.6 + 0.01
      {
        objects_storage.erase(objects_storage.begin());
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb1200_object_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get data from ROS param server
  if (!node_handle.getParam("/irb1200_objects_publisher/conveyor_velocity", conveyor_vel))
  {
    conveyor_vel = 0.1;
    ROS_WARN("Conveyor velocity is not defined and set to default: %f", conveyor_vel);
  }

  // Construct planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Wait for starting up the planning scene
  ros::WallDuration(3.0).sleep();

  // Add collision objects
  addCollisionObjects(planning_scene_interface);
  
  //
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.loadRemoteControl();
  // visual_tools.prompt("Press 'next' add an object");

  // Start a 100Hz timer to update object position
  ros::Timer timer = node_handle.createTimer(ros::Duration(timer_duration), TimerCallBack);
  ROS_INFO("100Hz Timer has started.");

  // Object pose publisher
  ros::Publisher object_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("moving_object", 1);

  // Visualization
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  std::stringstream str_conveyor_vel;
  str_conveyor_vel << "Conveyor speed: " << conveyor_vel << " m/s";

  //
  ros::Subscriber object_poses_subscriber = node_handle.subscribe("object_detection_cv", 1, ObjectPosesCallBack);

  //
  ros::Subscriber robot_done = node_handle.subscribe("robot_done", 1, DoneCallback);

  // while(ros::ok())
  // {
  //   switch (received_obj){
  //     case 0:
  //       break;
  //     case 1:
  //       while(objects_storage[0].position.y > -1)
  //       {
  //         // Publish current object pose
  //         // if (objects_storage[0].position.y > 0.6)
  //           object_pose_publisher.publish(objects_storage[0]);
          
          
  //         // Visualization
  //         visual_tools.deleteAllMarkers();
  //         for (std::size_t i = 0; i < objects_storage.size(); ++i)
  //           {
  //             visual_tools.publishAxis(objects_storage[i], 0.1);
  //           }
  //         visual_tools.publishText(text_pose, str_conveyor_vel.str(), rvt::WHITE, rvt::XLARGE);
  //         visual_tools.trigger();

  //         // Try to run the loop at 100 Hz
  //         ros::Rate(100).sleep();

  //         // if (objects_storage[0].position.y <= -0.5)
  //         //   objects_storage.erase(objects_storage.begin());
  //       }
  //       break;
  //   }
  // }

  while(ros::ok())
  {
    visual_tools.deleteAllMarkers();

    if (objects_storage.size() > 0)
    {
      if (objects_storage[0].position.y > -0.5)
      {
        // Publish current object pose
        object_pose_publisher.publish(objects_storage[0]);

        // Visualization
        for (std::size_t i = 0; i < objects_storage.size(); ++i)
        {
          visual_tools.publishAxis(objects_storage[i], 0.1);
        }
      }
      else
        objects_storage.erase(objects_storage.begin());
    }
    visual_tools.publishText(text_pose, str_conveyor_vel.str(), rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Try to run the loop at 100 Hz
    ros::Rate(100).sleep();
  }

  return 0;
}