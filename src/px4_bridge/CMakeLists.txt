cmake_minimum_required(VERSION 3.0.2)
project(px4_bridge)


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
  ../Utils
  ../control/DF/include

)

add_executable(statesToFcu_node
  include/StatesToFcu.h
  src/StatesToFcu.cpp
)
target_link_libraries(statesToFcu_node 
  ${catkin_LIBRARIES}
)

add_executable(statesFromFcu_node
  include/StatesFromFcu.h
  src/StatesFromFcu.cpp
)
target_link_libraries(statesFromFcu_node 
  ${catkin_LIBRARIES}
)

add_executable(commandToFcu_node
  include/CommandToFcu.h
  src/CommandToFcu.cpp
)
target_link_libraries(commandToFcu_node 
  ${catkin_LIBRARIES}
)

add_executable(commandToFcuAtt_node
  include/CommandToFcuAtt.h
  src/CommandToFcuAtt.cpp
)
target_link_libraries(commandToFcuAtt_node 
  ${catkin_LIBRARIES}
)

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )