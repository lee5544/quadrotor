cmake_minimum_required(VERSION 3.0.2)
project(can_rec)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)


add_message_files(FILES radar.msg radar_pkg.msg trigger.msg)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_radar
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(can_new src/can_new.cpp)
target_link_libraries(can_new ${catkin_LIBRARIES})
add_dependencies(can_new ${PROJECT_NAME}_generate_messages_cpp)


add_executable(can_alarm src/can_alarm.cpp)
target_link_libraries(can_alarm ${catkin_LIBRARIES})
add_dependencies(can_alarm ${PROJECT_NAME}_generate_messages_cpp)

add_executable(can_radar_show src/can_radar_show.cpp)
target_link_libraries(can_radar_show ${catkin_LIBRARIES})
add_dependencies(can_radar_show ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(can_mr72_filter src/can_mr72_filter.cpp)
# target_link_libraries(can_mr72_filter ${catkin_LIBRARIES})
# add_dependencies(can_mr72_filter ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(can_mr72_filter_alarm src/can_mr72_filter_alarm.cpp)
# target_link_libraries(can_mr72_filter_alarm ${catkin_LIBRARIES})
# add_dependencies(can_mr72_filter_alarm ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(filter_radar_show src/filter_radar_show.cpp)
# target_link_libraries(filter_radar_show ${catkin_LIBRARIES})
# add_dependencies(filter_radar_show ${PROJECT_NAME}_generate_messages_cpp)



