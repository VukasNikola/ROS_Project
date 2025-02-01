cmake_minimum_required(VERSION 3.0.2)
project(assignment2)

## Compile as C++11 (required for many ROS packages)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  apriltag_ros
  move_base_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
)

## Find MoveIt! planning interface package
find_package(moveit_ros_planning_interface REQUIRED)


## Declare catkin package
catkin_package(
  CATKIN_DEPENDS actionlib apriltag_ros move_base_msgs roscpp sensor_msgs tf tf2 tf2_geometry_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MOVEIT_ROS_PLANNING_SCENE_INTERFACE_INCLUDE_DIRS}  # This variable is set by MoveIt!
)

## Declare the table_navigation executable
add_executable(table_navigation src/table_navigation.cpp)
target_link_libraries(table_navigation
  ${catkin_LIBRARIES}
)

## Declare the collision_objects executable
add_executable(collision_objects src/collision_objects.cpp)
target_link_libraries(collision_objects 
  ${catkin_LIBRARIES}
)

