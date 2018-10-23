cmake_minimum_required(VERSION 2.8.3)
project(devine_head_coordinator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  tf
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  apriltags2_ros
  devine_config
  devine_irl_control
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
