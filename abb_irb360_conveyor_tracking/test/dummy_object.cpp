/* Author: Phat Do */
/*
  Target: Create random objects
*/

#include <moveit_visual_tools/moveit_visual_tools.h>

int obj_index = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_object");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 
  static const std::string BASE_FRAME = "base_link";
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.loadRemoteControl();

  // Object pose publisher
  ros::Publisher object_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("object_detection_cv", 1);

  geometry_msgs::Pose object_pose;

  const double x_max = 0.66;
  const double x_min = 0.33;

  const double z_max = 0.15;
  const double z_min = -0.02;

  while (ros::ok())
  {
    // visual_tools.prompt("Press 'next' add an object");
    object_pose.position.x = x_min + (rand() % 999 + 1)*(x_max - x_min)/1000;
    object_pose.position.y = 1.6;
    object_pose.position.z = z_min + (rand() % 999 + 1)*(z_max - z_min)/1000;  // the conveyor belt is lower than robot base 0.025 m
    object_pose.orientation.w = 1;
    object_pose_publisher.publish(object_pose);
    ROS_INFO("Object position %d: x = %f y = %f z = %f", obj_index++, object_pose.position.x, object_pose.position.y, object_pose.position.z);
    ros::Duration(1).sleep();
  }

  return 0;
}