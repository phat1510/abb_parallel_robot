/* Author: Phat Do */
/* Description:  
Check the effective robot workspace
*/

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
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/GetMotionSequence.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Utility functions for displaying and debugging data in Rviz via published markers
namespace rvt = rviz_visual_tools;

// Define planning group, base frame and end effector link
static const std::string PLANNING_GROUP = "manipulator";
static const std::string BASE_FRAME = "base_link";
static const std::string EE_BASE_LINK = "link_6"; 
static const std::string TOOL_LINK = "tool0"; // Working point

//------------------------------//
// Add some collision objects
//------------------------------//
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  int object_number = 1;
  collision_objects.resize(object_number);

  // Define conveyor
  const double conveyor_width = 0.5;
  const double conveyor_length = 3.0;
  const double conveyor_height = 0.129;
  collision_objects[0].id = "conveyor";
  collision_objects[0].header.frame_id = BASE_FRAME;
  // Define conveyor dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = conveyor_width; // conveyor width
  collision_objects[0].primitives[0].dimensions[1] = conveyor_length; // conveyor length
  collision_objects[0].primitives[0].dimensions[2] = conveyor_height; // conveyor height
  // Define conveyor pose in the robot frame
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = -1.0;
  collision_objects[0].primitive_poses[0].position.z = conveyor_height/2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //
  ROS_INFO("Add conveyor into the world");
  collision_objects[0].operation = collision_objects[0].ADD;

  //
  planning_scene_interface.addCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  // 
  ros::init(argc, argv, "irb1200_robot_calibration");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Add collision object: conveyor
  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  // 
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.loadRemoteControl();
  visual_tools.prompt("Press 'next' to start calibration");

  // ------------------------------------------------------------------//
  //-----------Setup robot model and planning pipeline
  // ------------------------------------------------------------------//

  // Load the robot description on ROS parameter server
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  
  // Construct a robot model, which contains the robot's kinematic info
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Construct a planning scene from the robot model loader
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

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
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  visual_tools.publishText(text_pose, "IRB1200 Robot Calibration", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Start the demo
  ROS_INFO("Setup finished. Ready to start...");

  // ------------------------------------------------------------------//
  //----------- X MAX
  // ------------------------------------------------------------------//
  
  // Get current joint position directly from  /joint_states topic and set for robot start state
  sensor_msgs::JointStateConstPtr joint_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  robot_state->setJointGroupPositions(joint_model_group,joint_values->position);

  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // Home position (bin's center position)
  const double gripper_height = 0.18;
  geometry_msgs::PoseStamped home_pose;
  home_pose.header.frame_id = "base_link";
  home_pose.pose.position.x = 0.701; // < 0.701
  home_pose.pose.position.y = 0;
  //home_pose.pose.position.z = 0.1 + gripper_height;
  home_pose.pose.position.z = 0.399 - 0.082;
  tf2::Quaternion orientation;
  orientation.setRPY(0, tau/4, 0); // tau = 2*pi;
  home_pose.pose.orientation = tf2::toMsg(orientation);

  // Define name of planning group
  req.group_name = "manipulator";
  req.planner_id = "PTP";
  req.allowed_planning_time = 5;
  req.max_velocity_scaling_factor = 0.1;
  req.max_acceleration_scaling_factor = 1;
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Set constraints for pose goal
  ROS_INFO("Construct goal constraints.");
  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);
  
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
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

  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  // joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  // visual_tools.trigger();
  //visual_tools.prompt("Press 'next' to execute");

  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // ------------------------------------------------------------------//
  //----------- Check X and Z ranges
  // ------------------------------------------------------------------//

  // Next pose
  //visual_tools.prompt("Press 'next' to go to x min");
  req.planner_id = "LIN";
  home_pose.pose.position.x -= 0.05;
  ROS_INFO("x = %f", home_pose.pose.position.x);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Next pose
  home_pose.pose.position.x = 0.3;
  ROS_INFO("x = %f", home_pose.pose.position.x);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    
  // Next pose
  // Z max
  //visual_tools.prompt("Press 'next' to go to z max");
  home_pose.pose.position.z += 0.2;
  ROS_INFO("z = %f", home_pose.pose.position.z);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Next pose
  // Z max - X max
  //visual_tools.prompt("Press 'next' to go to z max and x max");
  home_pose.pose.position.x = 0.66;
  ROS_INFO("x = %f", home_pose.pose.position.x);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();

  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Next pose
  // Z min - X max
  //visual_tools.prompt("Press 'next' to go to x max, z min");
  req.planner_id = "PTP";
  home_pose.pose.position.x = 0.701;
  home_pose.pose.position.z = 0.399 - 0.082;
  ROS_INFO("x = %f", home_pose.pose.position.x);
  ROS_INFO("z = %f", home_pose.pose.position.z);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // ------------------------------------------------------------------//
  //----------------------------- Revolve
  // ------------------------------------------------------------------//
  // 
  //visual_tools.prompt("Press 'next' to revolve");
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> _joint_values;
  goal_state.copyJointGroupPositions(joint_model_group, _joint_values);
  _joint_values[0] = -tau/4;
  //_joint_values = {0, 0, 0, 0, 0, 0};
  goal_state.setJointGroupPositions(joint_model_group, _joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.planner_id = "PTP";
  req.max_velocity_scaling_factor = 0.1;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Visualize the trajectory 
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK), joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);


  // ------------------------------------------------------------------//
  //----------- Y and Z ranges
  // ------------------------------------------------------------------//
  // Next pose
  // Z max - Y max
  //visual_tools.prompt("Press 'next' to go to x max, z min");
  home_pose.pose.position.x = 0;
  home_pose.pose.position.y = -0.66;
  home_pose.pose.position.z += 0.2;
  ROS_INFO("y = %f", home_pose.pose.position.y);
  ROS_INFO("z = %f", home_pose.pose.position.z);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
  pose_goal = kinematic_constraints::constructGoalConstraints("link_6", home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  req.planner_id = "PTP";
  req.max_velocity_scaling_factor = 0.1;
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }
  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Get response message
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // ------------------------------------------------------------------//
  //----------- Revolve
  // ------------------------------------------------------------------//
  // 
  moveit::core::RobotState _goal_state(*robot_state);
  //visual_tools.prompt("Press 'next' to revolve");
  _goal_state.copyJointGroupPositions(joint_model_group, _joint_values);
  _joint_values[0] = 0;
  _goal_state.setJointGroupPositions(joint_model_group, _joint_values);
  joint_goal = kinematic_constraints::constructGoalConstraints(_goal_state, joint_model_group);
  req.planner_id = "PTP";
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful
    planning_pipeline->generatePlan(lscene, req, res);
  }
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Visualize the trajectory 
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK), joint_model_group);
  visual_tools.trigger();
  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // -------------------------------------------------------------------
  visual_tools.prompt("Press 'next' to stop calibration");
  ros::shutdown();
  return 0;
}