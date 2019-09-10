cmake_minimum_required(VERSION 2.8.3)
project(auvsim_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auvsim_gazebo
  CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
  DEPENDS gazebo_ros roscpp rospy std_msgs
)

