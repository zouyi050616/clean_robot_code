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
  zoo_bringup
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(movebase_client src/movebase_client.cpp)
target_link_libraries(movebase_client ${catkin_LIBRARIES})

add_executable(tf_echo_node src/tf_echo.cpp)
target_link_libraries(tf_echo_node ${catkin_LIBRARIES})

add_executable(arm_grab_node src/arm_grab.cpp)
target_link_libraries(arm_grab_node ${catkin_LIBRARIES})

add_executable(arm_put_node src/arm_put.cpp)
target_link_libraries(arm_put_node ${catkin_LIBRARIES})