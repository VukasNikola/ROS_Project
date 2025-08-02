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
  GetObjectPose.srv
  GetAllObjectPoses.srv
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

add_executable(test_node src/test_node.cpp)
add_executable(node_A src/node_A.cpp)
add_executable(node_B src/node_B.cpp)
add_executable(node_C src/node_C.cpp)


target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(node_A ${catkin_LIBRARIES})
target_link_libraries(node_B ${catkin_LIBRARIES})
target_link_libraries(node_C ${catkin_LIBRARIES})


## Ensure message/service targets are built before your nodes
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_A ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_B ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_C ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
