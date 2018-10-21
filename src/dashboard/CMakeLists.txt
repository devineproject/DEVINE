cmake_minimum_required(VERSION 2.8.3)
project(devine_dashboard)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  devine_config
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
