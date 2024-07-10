cmake_minimum_required(VERSION 3.0.2)
project(roscomm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES roscomm_serial.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
