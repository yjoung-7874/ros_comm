cmake_minimum_required(VERSION 3.0.2)
project(roscomm_serial)

## Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CMakePrintHelpers)

cmake_print_variables(SERIAL_LIBRARY_PATH)
list(APPEND CMAKE_PREFIX_PATH ${SERIAL_LIBRARY_PATH})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roscomm_msgs
  serial
  ros_wrapper
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscomm_msgs std_msgs serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(
  ${PROJECT_NAME}_node
  src/roscomm_serial_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY migration_rules scripts config launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  USE_SOURCE_PERMISSIONS)
