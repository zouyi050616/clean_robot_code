cmake_minimum_required(VERSION 3.0.2)
project(clean_desktop_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(movebase_client src/movebase_client.cpp)
target_link_libraries(movebase_client ${catkin_LIBRARIES})

add_executable(tf_echo_node src/tf_echo.cpp)
target_link_libraries(tf_echo_node ${catkin_LIBRARIES})