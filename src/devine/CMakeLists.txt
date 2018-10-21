cmake_minimum_required(VERSION 2.8.3)
project(devine)

find_package(catkin REQUIRED
  devine_common
  devine_config
  devine_dashboard
  devine_dialog
  devine_guesswhat
  devine_image_processing
  devine_irl_control
  openni_camera
  openni_launch
)
