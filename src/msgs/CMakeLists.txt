cmake_minimum_required(VERSION 2.8.3)
project(prometheus_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  # DIRECTORY msg
  FILES
  ControlCommand.msg
  PositionReference.msg
  DroneState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  # roscpp
  # geometry_msgs
  # std_msgs
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
