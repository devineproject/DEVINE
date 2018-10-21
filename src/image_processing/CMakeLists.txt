cmake_minimum_required(VERSION 2.8.3)
project(devine_image_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  devine_config
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
