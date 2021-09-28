cmake_minimum_required(VERSION 3.0.2)
project(mission)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  prometheus_msgs
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS  message_runtime
#  LIBRARIES mavros_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)



add_executable(missionSample_node
  test/MissionSample_node.cpp
  include/MissionSample.h
  src/MissionSample.cpp)
target_link_libraries(missionSample_node
  ${catkin_LIBRARIES})

add_executable(polygonMission_node
  test/PolygonMission_node.cpp
  include/PolygonMission.h
  src/PolygonMission.cpp)
target_link_libraries(polygonMission_node
  ${catkin_LIBRARIES})

  add_executable(missionRtk_node
  test/MissionRtk_node.cpp
  include/MissionRtk.h
  src/MissionRtk.cpp)
target_link_libraries(missionRtk_node
  ${catkin_LIBRARIES})
# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
