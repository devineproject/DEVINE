cmake_minimum_required(VERSION 2.4.6)
project(devine_irl_control)

find_package(catkin REQUIRED COMPONENTS 
   rospy 
   actionlib 
   std_msgs 
   sensor_msgs 
   control_msgs 
   trajectory_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
)