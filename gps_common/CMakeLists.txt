cmake_minimum_required(VERSION 2.8.3)
project(gps_common CXX)

############
## Catkin ##
############

find_package(catkin REQUIRED
  message_generation
  message_filters
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES std_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}/utm_odometry_node src/utm_odometry_node.cpp)
set_target_properties(${PROJECT_NAME}/utm_odometry_node PROPERTIES OUTPUT_NAME "utm_odometry_node")
target_link_libraries(${PROJECT_NAME}/utm_odometry_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}/utm_odometry_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}/utm_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
