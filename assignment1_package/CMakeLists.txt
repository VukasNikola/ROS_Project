cmake_minimum_required(VERSION 3.0.2)
project(assignment1_package)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  apriltag_ros
  move_base_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros        
  geometry_msgs  
  std_msgs      
  message_generation
  control_msgs
  trajectory_msgs
)

## Declare ACTIONS
add_action_files(
  DIRECTORY action
  FILES
    NavigationTask.action
)

## Generate code for msgs/srvs/actions
generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs   
)

catkin_package(
  CATKIN_DEPENDS
  roscpp actionlib move_base_msgs tf2 tf2_geometry_msgs tf2_ros
  message_runtime actionlib_msgs geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_head_down_node src/move_head_down_node.cpp)
add_dependencies(move_head_down_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_head_down_node ${catkin_LIBRARIES})

add_executable(node_A src/node_A.cpp)
add_dependencies(node_A ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_A ${catkin_LIBRARIES}
)

add_executable(node_B src/node_B.cpp)
add_dependencies(node_B ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_B ${catkin_LIBRARIES}
)
