cmake_minimum_required(VERSION 2.8.3)
project(auvsim_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(freebuoyancy src/freebuoyancy.cpp)
target_link_libraries(freebuoyancy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(auv_plugin src/auv_plugin.cpp)
target_link_libraries(auv_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_message_files(
   FILES
   ThrusterSpeeds.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auvsim_gazebo
  CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
  DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime
)
