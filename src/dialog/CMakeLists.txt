cmake_minimum_required(VERSION 2.8.3)
project(devine_dialog)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  devine_config
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  TtsQuery.msg
  TtsAnswer.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

configure_file(src/devine_dialog/static/dialogs.json
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/static/dialogs.json
  COPYONLY
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)
