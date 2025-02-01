cmake_minimum_required(VERSION 3.0.2)
project(assignment2_package)

## Set C++ standard to C++14
add_compile_options(-std=c++14)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  message_generation
  tiago_iaslab_simulation
  apriltag_ros
  actionlib
)

## (If you have your own service files, list them here)
add_service_files(
  FILES
  PickObject.srv
  PlaceObject.srv
)

## Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs moveit_ros_planning_interface tf2_ros tf2_geometry_msgs message_runtime tiago_iaslab_simulation apriltag_ros actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executables for your nodes
add_executable(coordinator_node src/coordinator_node.cpp)
add_executable(detection_collision_node src/detection_collision_node.cpp)
add_executable(manipulation_node src/manipulation_node.cpp)
add_executable(move_head_down_node src/move_head_down_node.cpp)

target_link_libraries(coordinator_node ${catkin_LIBRARIES})
target_link_libraries(detection_collision_node ${catkin_LIBRARIES})
target_link_libraries(manipulation_node ${catkin_LIBRARIES})
target_link_libraries(move_head_down_node ${catkin_LIBRARIES})

## Ensure message/service targets are built before your nodes
add_dependencies(coordinator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detection_collision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_head_down_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
