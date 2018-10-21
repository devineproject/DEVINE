cmake_minimum_required(VERSION 2.8.3)
project(devine_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  devine_config
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()
