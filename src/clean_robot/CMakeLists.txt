cmake_minimum_required(VERSION 3.0.2)
project(clean_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  apriltag_ros
  upros_core_move
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_generate_node src/path_generate.cpp)
target_link_libraries(path_generate_node ${catkin_LIBRARIES})

add_executable(three_path_generate_node src/three_path_generate.cpp)
target_link_libraries(three_path_generate_node ${catkin_LIBRARIES})

add_executable(move_base_client src/movebase_client.cpp)
target_link_libraries(move_base_client ${catkin_LIBRARIES})

add_executable(core_move_client src/coremove_client.cpp)
target_link_libraries(core_move_client ${catkin_LIBRARIES})

add_executable(apriltag_detect src/apriltag_detect.cpp)
target_link_libraries(apriltag_detect ${catkin_LIBRARIES})

add_executable(area_path_node src/AreaPathNode.cpp src/AreaPath.cpp)
target_link_libraries(area_path_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(next_goal src/next_goal.cpp)
target_link_libraries(next_goal ${catkin_LIBRARIES})