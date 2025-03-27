cmake_minimum_required(VERSION 3.0.2)
project(shoot_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(shoot_node src/shoot.cpp)
target_link_libraries(shoot_node ${catkin_LIBRARIES})