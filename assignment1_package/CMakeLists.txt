cmake_minimum_required(VERSION 3.0.2)
project(assignment1_package)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  apriltag_ros
  move_base_msgs
  roscpp
  sensor_msgs
  tf
  roscpp
  tf2
  tf2_geometry_msgs
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib move_base_msgs tf2 tf2_geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(waypoints_navigation_node src/waypoints_navigation_node.cpp)
target_link_libraries(waypoints_navigation_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
add_executable(move_head_down_node src/move_head_down_node.cpp)
target_link_libraries(move_head_down_node
  ${catkin_LIBRARIES}
)

add_executable(node_A src/node_A.cpp)
target_link_libraries(node_A
  ${catkin_LIBRARIES}
)
add_executable(node_B src/node_B.cpp)
target_link_libraries(node_B
  ${catkin_LIBRARIES}
)
