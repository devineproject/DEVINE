cmake_minimum_required(VERSION 2.8.3)
project(devine_head_coordinator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  tf
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  apriltags2_ros
  devine_config
  devine_irl_control
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES LookAtHuman.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
