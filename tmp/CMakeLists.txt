cmake_minimum_required(VERSION 2.8.3)
project(intera_examples)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface
  geometry_msgs
  message_generation
)

catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SortableObjectMessage.msg
  PoseGrippMessage.msg
)


generate_dynamic_reconfigure_options(
  cfg/SawyerJointSpringsExample.cfg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  intera_core_msgs
  geometry_msgs
  trajectory_msgs
  intera_motion_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  geometry_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface
  message_runtime
  message_generation
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
