cmake_minimum_required(VERSION 2.8.3)
project(devine_guesswhat)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  devine_config
  devine_dialog
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
