cmake_minimum_required(VERSION 2.8.3)
project(devine_image_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  devine_config
  message_generation
)

add_message_files(
  FILES
  SceneObject.msg
  SegmentedImage.msg
  VGG16Features.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)
