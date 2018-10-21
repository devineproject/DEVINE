cmake_minimum_required(VERSION 2.8.3)
project(devine_game_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  trajectory_msgs
  control_msgs
  devine_config
  devine_common
  devine_dialog
  devine_irl_control
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()
