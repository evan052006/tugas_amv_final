cmake_minimum_required(VERSION 3.0.2)
project(tugas_amv_final)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BuoyCount.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tugas_amv_final
#  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge image_transport rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/Cam.py
  scripts/Utama.py
  scripts/Movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
