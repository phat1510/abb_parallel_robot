/* Author: Phat Do */
/* Description:  
Robot: ABB IRB1200 7/70
Planner: Pilz industrial motion planner
Function: Pick a moving object requested from irb1200_objects_publisher client
*/

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group/move_group_capability.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/GetMotionSequence.h>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Bool.h>

namespace rvt = rviz_visual_tools;

class ConveyorTracking
{
  private:

    ros::NodeHandle node_handle;

    //
    const std::string DELTA_IRB360 = "delta_irb360";
    const std::string DELTA_ARM_1 = "delta_arm_1";
    const std::string DELTA_ARM_2 = "delta_arm_2";
    const std::string DELTA_ARM_3 = "delta_arm_3";
    const std::string BASE_FRAME = "base_link";
    const std::string TOOL_LINK_1 = "arm_1_tool0";
    const std::string TOOL_LINK_2 = "arm_2_tool0";
    const std::string TOOL_LINK_3 = "arm_3_tool0";  
    const std::string PLANNING_GROUP = DELTA_IRB360;
    const std::string TOOL_LINK = TOOL_LINK_3;


    const double tau = 2 * M_PI;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    moveit::core::RobotStatePtr robot_state;
    moveit::core::RobotModelPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface move_group_interface;

    // Sequence of trajectories variables
    planning_interface::MotionPlanRequest first_req;
    planning_interface::MotionPlanRequest followed_req;
    moveit_msgs::MotionSequenceRequest seq_req;
    moveit_msgs::MotionSequenceResponse seq_res;
    moveit_msgs::MotionSequenceItem sequence_item;
    moveit_msgs::RobotTrajectory sequence_traj;
    moveit_msgs::GetMotionSequence planner_srv;

    // Service client for sequence of trajectories planning
    ros::ServiceClient _planner_client = node_handle.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");

    // Feedback topic
    ros::Publisher robot_done = node_handle.advertise<std_msgs::Bool>("robot_done", 10);
    std_msgs::Bool done;

    // Planning constraint vectors
    std::vector<double> tolerance_pose;
    std::vector<double> tolerance_angle;
    moveit_msgs::Constraints pose_goal;

    // IRB1200 working parameters
    tf2::Quaternion robot_orientation;

    double exe_time;

    const double max_lin_vel = 1.5;
    const double max_conv_vel = 1.2;
    const double down_vel = 0.8;
    const double pick_up_vel = 0.8;
    const double height_offset = 0.1;
    const double quick_moving_vel_factor = 0.8;
    const double limit_radius = 0.673; // this is the maximum radius robot can reach at the conveyor plane (z = -0.025 m)

    double working_conveyor_vel;
    double gripping_time;
    double offset_tracking_point;

    // Store current object position
    geometry_msgs::Pose object_pose;

  public:
    /** \brief Initialize planning scene
     * 
     * 
     * 
    * */
    ConveyorTracking(): 
      robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description")),
      planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader)),
      robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentState())),
      move_group_interface(PLANNING_GROUP),
      visual_tools(BASE_FRAME)
      {
        robot_model = robot_model_loader->getModel();
        joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
        ROS_INFO("Setup completed.");
      }

    /** \brief A useful class to visualize and debug code
     * 
     * 
     * 
    * */
    moveit_visual_tools::MoveItVisualTools visual_tools;

    /** \brief Setup initial conditions
     * 
     * 
     * 
    * */
    void Init()
    {
      ROS_INFO("Load parameters on ROS server.");
      LoadParameters();
      
      // Define planning tolerances
      tolerance_pose = {0.01, 0.01, 0.01};
      tolerance_angle = {0.01, 0.01, 0.01};

      // Some members of planning request need to config once
      first_req.group_name = PLANNING_GROUP;
      first_req.max_acceleration_scaling_factor = 1.0;
      first_req.allowed_planning_time = 1.0;
      followed_req.group_name = PLANNING_GROUP;
      followed_req.max_acceleration_scaling_factor = 1.0;
      followed_req.allowed_planning_time = 1.0;

      // Visualization
      visual_tools.deleteAllMarkers();
      visual_tools.loadRemoteControl();
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 0.9;
      visual_tools.publishText(text_pose, "HOME POSITION", rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();

      done.data = true;
    }

    /** \brief Load data from ROS parameters server. If values are not available on ROS server, they will be set to default
     * 
     * 
     * 
    * */
    void LoadParameters()
    {
      if (!node_handle.getParam("/irb1200_objects_publisher/conveyor_velocity", working_conveyor_vel))
      {
        working_conveyor_vel = 0.1;
        ROS_WARN("Conveyor velocity is not defined and set to default: %f", working_conveyor_vel);
      }

      if (!node_handle.getParam("/robot_conveyor_tracking/gripping_time", gripping_time))
      {
        gripping_time = 0.35;
        ROS_WARN("Gripping time is not defined and set to default: %f", gripping_time);
      }

      if (!node_handle.getParam("/robot_conveyor_tracking/tracking_point_y_offset", offset_tracking_point))
      {
        offset_tracking_point = 0.11;
        ROS_WARN("y offset for tracking point is not defined and set to default: %f", offset_tracking_point);
      }

    }

    /** \brief Read current robot pose and move to home position. Plan and execute immediately.
     * 
     * 
    * */
    bool Homing()
    {
      // ------------------------------------------------------------------//
      // Setup home position
      // ------------------------------------------------------------------//

      // Get current joint position directly from  /joint_states topic and set for robot start state
      sensor_msgs::JointStateConstPtr joint_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
      robot_state->setJointGroupPositions(joint_model_group,joint_values->position);

      // Home position
      geometry_msgs::PoseStamped home_pose;
      home_pose.header.frame_id = "base_link";
      home_pose.pose.position.x = 0.55;
      home_pose.pose.position.y = 0;
      home_pose.pose.position.z = 0.3;
      robot_orientation.setRPY(0, tau/4, 0);
      home_pose.pose.orientation = tf2::toMsg(robot_orientation);

      // Go to home using point-to-point planning
      first_req.planner_id = "PTP";

      // 20% of max joint velocity
      first_req.max_velocity_scaling_factor = 0.2; 

      // Setup start state. Only the first request of a sequence has a start state.
      moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);

      // Set constraints for pose goal
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, home_pose, tolerance_pose, tolerance_angle);
      first_req.goal_constraints.clear();
      first_req.goal_constraints.push_back(pose_goal);

      // Add to sequence item vector
      sequence_item.req = first_req;
      sequence_item.blend_radius = 0;
      seq_req.items.push_back(sequence_item);

      // Assign client request and call service
      planner_srv.request.request = seq_req;
      if (_planner_client.call(planner_srv))
      {
        sequence_traj = planner_srv.response.response.planned_trajectories.back();
        ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
        // visual_tools.publishTrajectoryLine(sequence_traj, 
        //   joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
        // visual_tools.trigger();
        
        // Execute the sequence and reset joint states
        exe_time = ros::Time::now().toSec();
        move_group_interface.execute(sequence_traj);
        exe_time = ros::Time::now().toSec() - exe_time;
        ROS_INFO("Execution time = %fs", exe_time);
      }
      else
      {
        ROS_ERROR("Failed to call service plan_sequence_path");
        return false;
      }

      // Lock the planning scene and read robot's current state
      robot_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
      robot_state->setJointGroupPositions(joint_model_group, 
        planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);
      
      return true;
    }

    /** \brief ...
     * 
    * */
    bool GenerateTrackingPath()
    {
      // ------------------------------------------------------------------//
      // Go to start point of tracking line
      // ------------------------------------------------------------------//
      
      ROS_INFO("Construct tracking point goal");
      first_req.planner_id = "PTP";
      geometry_msgs::PoseStamped tracking_point;
      tracking_point.header.frame_id = "base_link";
      tracking_point.pose.position.x = object_pose.position.x; // get the x coordinate of object
      tracking_point.pose.position.y = 
        sqrt(limit_radius * limit_radius - tracking_point.pose.position.x * tracking_point.pose.position.x); 
      if (tracking_point.pose.position.x < 0.6)
      {
        tracking_point.pose.position.y = sqrt(limit_radius * limit_radius - 0.6 * 0.6);
      }
      tracking_point.pose.position.z = object_pose.position.z + height_offset;
      robot_orientation.setRPY(0, tau/4, 0); // tau = 2*pi;
      tracking_point.pose.orientation = tf2::toMsg(robot_orientation);
      ROS_INFO("Tracking point x = %f y = %f z = %f.", 
        tracking_point.pose.position.x, tracking_point.pose.position.y, tracking_point.pose.position.z);
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, tracking_point, tolerance_pose, tolerance_angle);
      first_req.goal_constraints.clear();
      first_req.goal_constraints.push_back(pose_goal);
      moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);
      first_req.max_velocity_scaling_factor = quick_moving_vel_factor;

      // Add to sequence item vector
      sequence_item.req = first_req;
      sequence_item.blend_radius = 0; // in meter
      seq_req.items.clear();
      seq_req.items.push_back(sequence_item);

      // Assign client request and call service
      planner_srv.request.request = seq_req;
      if (_planner_client.call(planner_srv))
      {
        sequence_traj = planner_srv.response.response.planned_trajectories.back();
        ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
        // visual_tools.publishTrajectoryLine(sequence_traj, 
        //   joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
        // visual_tools.trigger();

        // Execute the sequence and reset joint states
        exe_time = ros::Time::now().toSec();
        move_group_interface.execute(sequence_traj);
        exe_time = ros::Time::now().toSec() - exe_time;
        ROS_INFO("Execution time = %fs", exe_time);
      }
      else
      {
        ROS_ERROR("Failed to call service plan_sequence_path");
        return false;
      }

      // Update the robot state
      robot_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
      robot_state->setJointGroupPositions(joint_model_group, 
      planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);
      
      // ------------------------------------------------------------------//
      // Plan a sequence of paths
      // ------------------------------------------------------------------//

      // Initialize first planning request. Only this request has a start_state
      first_req.planner_id = "LIN";
      first_req.max_velocity_scaling_factor = working_conveyor_vel / max_lin_vel;

      // First waypoint: follow and synchronize 
      ROS_INFO("Construct start picking goal");
      geometry_msgs::PoseStamped next_pose;
      next_pose = tracking_point;
      next_pose.pose.position.y -= 0.02;
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
      first_req.goal_constraints.clear();
      first_req.goal_constraints.push_back(pose_goal);
      moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);
      // Add to sequence item vector
      sequence_item.req = first_req;
      sequence_item.blend_radius = 0; // in meter
      seq_req.items.clear();
      seq_req.items.push_back(sequence_item);

      // Next waypoint: down and grasp
      ROS_INFO("Construct down and grasp goal");
      followed_req.planner_id = "LIN";
      next_pose.pose.position.z -= height_offset;
      double t_vel_z = height_offset / down_vel; // not including acceleration factor
      double move_group_execution_delay = 0;
      next_pose.pose.position.y -= (t_vel_z + move_group_execution_delay) * working_conveyor_vel;
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
      followed_req.goal_constraints.clear();
      followed_req.goal_constraints.push_back(pose_goal);
      followed_req.max_velocity_scaling_factor = 
        sqrt(working_conveyor_vel * working_conveyor_vel + down_vel * down_vel)/max_lin_vel;
      // Add to sequence item vector
      sequence_item.req = followed_req;
      sequence_item.blend_radius = 0;
      seq_req.items.push_back(sequence_item);

      // Next waypoint: keep the height of gripper for a while
      ROS_INFO("Construct end of tracking goal");
      next_pose.pose.position.y -= gripping_time * working_conveyor_vel;
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
      followed_req.goal_constraints.clear();
      followed_req.goal_constraints.push_back(pose_goal);
      followed_req.max_velocity_scaling_factor = working_conveyor_vel/max_lin_vel;
      // Add to sequence item vector
      sequence_item.req = followed_req;
      sequence_item.blend_radius = 0;
      seq_req.items.push_back(sequence_item);

      // Next waypoint: Up with high speed
      ROS_INFO("Construct picking up goal");
      next_pose.pose.position.z += height_offset;
      pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
      followed_req.goal_constraints.clear();
      followed_req.goal_constraints.push_back(pose_goal);
      followed_req.max_velocity_scaling_factor = pick_up_vel/max_lin_vel;
      // Add to sequence item vector
      sequence_item.req = followed_req;
      sequence_item.blend_radius = 0;
      seq_req.items.push_back(sequence_item);

      // Next waypoint - to the bin
      ROS_INFO("Construct end goal");
      moveit::core::RobotState goal_state(*robot_state);
      std::vector<double> _joint_values;
      // _joint_values = {1.471128, 0.706638, 0.466189, 0.000000, 0.397970, 1.471128}; // x = 0.04, y = 0.4, z = 0.15
      _joint_values = {1.471128, 0.643702, 0.472227, 0.000000, 0.454867, 1.471128}; // x = 0.04, y = 0.4, z = 0.20
      goal_state.setJointGroupPositions(joint_model_group, _joint_values);
      pose_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
      // next_pose.pose.position.x = 0.04; // default = 0.85
      // next_pose.pose.position.y = 0.4; // +-0.22 for left and right bin
      // next_pose.pose.position.z += 0.05;
      // orientation.setRPY(0, 0, 0); // tau = 2*pi;
      // next_pose.pose.orientation = tf2::toMsg(orientation);
      // pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
      followed_req.goal_constraints.clear();
      followed_req.goal_constraints.push_back(pose_goal);
      followed_req.planner_id = "PTP";
      followed_req.max_velocity_scaling_factor = quick_moving_vel_factor;
      // Add to sequence item vector
      sequence_item.req = followed_req;
      sequence_item.blend_radius = 0;
      seq_req.items.push_back(sequence_item);

      // ------------------------------------------------------------------//
      //----------- Request sequence of multiple segments
      // ------------------------------------------------------------------//

      // Assign client request
      planner_srv.request.request = seq_req;

      if (_planner_client.call(planner_srv))
      {
        sequence_traj = planner_srv.response.response.planned_trajectories.back();
        ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
        // visual_tools.publishTrajectoryLine(sequence_traj, 
        //   joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
        // visual_tools.trigger();
      }
      else
      {
        ROS_ERROR("Failed to call service /plan_sequence_path");
        return false;
      }

      // ------------------------------------------------------------------------//
      // Wait for the object to come to tracking point for executing -  temporary solution
      // ------------------------------------------------------------------------//

      // There's a delay when execute a planned tracjectory, so we need to execute with an offset
      double actual_tracking_point = tracking_point.pose.position.y + offset_tracking_point;

      //
      while(object_pose.position.y > actual_tracking_point || 
        object_pose.position.y < (tracking_point.pose.position.y - 0.01))
      {
        ros::Rate(100).sleep();
      }

      ROS_INFO("Tracking start at y = %f", object_pose.position.y);

      // Execute the sequence and reset joint states
      exe_time = ros::Time::now().toSec();
      move_group_interface.execute(sequence_traj);
      exe_time = ros::Time::now().toSec() - exe_time;
      ROS_INFO("Execution time = %fs", exe_time);

      // Update the robot state
      robot_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
      robot_state->setJointGroupPositions(joint_model_group, 
        planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);

      visual_tools.deleteAllMarkers();

      robot_done.publish(done);

      return true;
    }

    bool CheckComingPose()
    {
      double x = object_pose.position.x;
      double y = object_pose.position.y;
      double z = object_pose.position.z;

      if ( x < 0.66 && x > 0.33 && y > 0.6 && z < 0.15) // default y > 0.6
        return true;
      else 
      {
        return false;
      }
    }

    void CopyJointValues()
    {
      // Copy reusable joint's values
      moveit::core::RobotState goal_state(*robot_state);
      std::vector<double> _joint_values;
      const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
      goal_state.copyJointGroupPositions(joint_model_group, _joint_values);
      for (int i = 0; i < joint_names.size(); i++)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), _joint_values[i]);
      }
    }

    void ObjectCallBack(const geometry_msgs::Pose::ConstPtr &msg)
    {
      object_pose.position = msg->position;
      object_pose.orientation = msg->orientation;
      //ROS_INFO("Point is updated %f", msg->position.y);
    }

    bool DeltaRobotMotionPlanning()
    {
      // ------------------------------------------------------------------//
      // Setup home position
      // ------------------------------------------------------------------//

      // Get current joint position directly from  /joint_states topic and set for robot start state
      // sensor_msgs::JointStateConstPtr joint_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
      // robot_state->setJointGroupPositions(joint_model_group,joint_values->position);

      // Home position
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "base_link";
      goal_pose.pose.position.x = 0.4;
      goal_pose.pose.position.y = 0;
      goal_pose.pose.position.z = -1.0;
      robot_orientation.setRPY(0, 0, 0);
      goal_pose.pose.orientation = tf2::toMsg(robot_orientation);

      // Go to home using point-to-point planning
      first_req.planner_id = "PTP";

      // 20% of max joint velocity
      first_req.max_velocity_scaling_factor = 0.2; 

      // Setup start state. Only the first request of a sequence has a start state.
      moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);

      // Set constraints for pose goal
      moveit::core::RobotState goal_state(*robot_state);
      std::vector<double> _joint_values;
      _joint_values = {-0.323334, -0.213630, -0.000000, 0.000003, 0.536961, 0.970480, -0.736802, 0.447826, 
    0.447830, -0.233681, 0.970481, -0.736806, -0.447827, -0.447823, -0.233677};
      goal_state.setJointGroupPositions(joint_model_group, _joint_values);
      pose_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      // pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, goal_pose, tolerance_pose, tolerance_angle);
      first_req.goal_constraints.clear();
      first_req.goal_constraints.push_back(pose_goal);

      // Add to sequence item vector
      sequence_item.req = first_req;
      sequence_item.blend_radius = 0;
      seq_req.items.clear();
      seq_req.items.push_back(sequence_item);

      // Assign client request and call service
      planner_srv.request.request = seq_req;
      if (_planner_client.call(planner_srv))
      {
        sequence_traj = planner_srv.response.response.planned_trajectories.back();
        ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
        // visual_tools.publishTrajectoryLine(sequence_traj, 
        //   joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
        // visual_tools.trigger();
        
        // Execute the sequence and reset joint states
        exe_time = ros::Time::now().toSec();
        move_group_interface.execute(sequence_traj);
        exe_time = ros::Time::now().toSec() - exe_time;
        ROS_INFO("Execution time = %fs", exe_time);
      }
      else
      {
        ROS_ERROR("Failed to call service plan_sequence_path");
        return false;
      }

      // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // move_group_interface.setPlannerId("LIN");
      // move_group_interface.setPoseTarget(goal_pose, TOOL_LINK);
      // move_group_interface.plan(my_plan);
      // move_group_interface.execute(my_plan);

      // Lock the planning scene and read robot's current state
      robot_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
      robot_state->setJointGroupPositions(joint_model_group, 
        planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);
      
      // Setup start state. Only the first request of a sequence has a start state.
      moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);

      ros::Duration(0.5).sleep();

      _joint_values = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      goal_state.setJointGroupPositions(joint_model_group, _joint_values);
      pose_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      // pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, goal_pose, tolerance_pose, tolerance_angle);
      first_req.goal_constraints.clear();
      first_req.goal_constraints.push_back(pose_goal);

      // Add to sequence item vector
      sequence_item.req = first_req;
      sequence_item.blend_radius = 0;
      seq_req.items.clear();
      seq_req.items.push_back(sequence_item);
      
      // Assign client request and call service
      planner_srv.request.request = seq_req;
      if (_planner_client.call(planner_srv))
      {
        sequence_traj = planner_srv.response.response.planned_trajectories.back();
        ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
        // visual_tools.publishTrajectoryLine(sequence_traj, 
        //   joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
        // visual_tools.trigger();
        
        // Execute the sequence and reset joint states
        exe_time = ros::Time::now().toSec();
        move_group_interface.execute(sequence_traj);
        exe_time = ros::Time::now().toSec() - exe_time;
        ROS_INFO("Execution time = %fs", exe_time);
      }
      else
      {
        ROS_ERROR("Failed to call service plan_sequence_path");
        return false;
      }

      // Lock the planning scene and read robot's current state
      robot_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
      robot_state->setJointGroupPositions(joint_model_group, 
        planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);

      ros::Duration(0.5).sleep();
      
      return true;
    }
};


int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "robot_conveyor_tracking");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();  

  // Create a conveyor tracking object for the robot
  ConveyorTracking abb_irb1200_7_70_robot;

  // Initialize parameters and configurations
  abb_irb1200_7_70_robot.Init();

  // Waypoint 1
  abb_irb1200_7_70_robot.visual_tools.prompt("Press next to go to next wp");
  // geometry_msgs::Pose wp;
  // wp.position.x = 0.1;
  // wp.position.y = 0.1;
  // wp.position.z = -1;
  // wp.orientation.w = 1;
  while (ros::ok())
  {
    abb_irb1200_7_70_robot.DeltaRobotMotionPlanning();
  }
  

  // // Move the robot from current position to home
  // abb_irb1200_7_70_robot.Homing();

  // // Subscribe 
  // ros::Subscriber object_pose_subscriber = 
  //   node_handle.subscribe("moving_object", 1, &ConveyorTracking::ObjectCallBack, &abb_irb1200_7_70_robot);

  // // I will remove these whiles by ROS action
  // while (ros::ok())
  // {

  //   while(!abb_irb1200_7_70_robot.CheckComingPose())
  //   {
  //     ros::Rate(100).sleep();
  //   }

  //   abb_irb1200_7_70_robot.GenerateTrackingPath();

  // }

  

  return 0;
}