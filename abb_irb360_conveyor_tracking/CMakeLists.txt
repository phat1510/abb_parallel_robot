cmake_minimum_required(VERSION 3.1.3)
project(abb_irb360_conveyor_tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES cr_waste_sorting_robot
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)
###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS})

# /src folder
add_executable(irb1200_conveyor_tracking src/irb1200_conveyor_tracking.cpp)
target_link_libraries(irb1200_conveyor_tracking ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(irb1200_objects_publisher src/irb1200_objects_publisher.cpp)
target_link_libraries(irb1200_objects_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(irb1200_robot_calibration src/irb1200_robot_calibration.cpp)
target_link_libraries(irb1200_robot_calibration ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(irb1200_robot_monitoring src/irb1200_robot_monitoring.cpp)
target_link_libraries(irb1200_robot_monitoring ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_conveyor_tracking src/robot_conveyor_tracking.cpp)
target_link_libraries(robot_conveyor_tracking ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# /test folder
add_executable(dummy_object test/dummy_object.cpp)
target_link_libraries(dummy_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_pilz_motion_planner test/test_pilz_motion_planner.cpp)
target_link_libraries(test_pilz_motion_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_irb1200_move_group_interface test/test_irb1200_move_group_interface.cpp)
target_link_libraries(test_irb1200_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})



