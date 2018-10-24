cmake_minimum_required(VERSION 2.8.3)
project(devine_irl_control)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  tf
  actionlib
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  jn0_face_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()
