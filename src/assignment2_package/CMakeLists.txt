cmake_minimum_required(VERSION 3.0.2)
project(assignment2_package)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin and all required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  message_generation
  apriltag_ros
  actionlib
  tiago_iaslab_simulation
)

## We also need the geometric_shapes library for mesh loading
find_package(geometric_shapes REQUIRED)

## Declare your own messages and services
add_service_files(
  FILES
    PickObject.srv
    GetObjectPose.srv
)
add_message_files(
  FILES
    ObjectPose.msg
    ObjectPoseArray.msg
)

## Generate message & service code
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

## Catkin exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    shape_msgs
    moveit_ros_planning_interface
    tf2_ros
    tf2_geometry_msgs
    message_runtime
    apriltag_ros
    actionlib
    tiago_iaslab_simulation
  DEPENDS
    geometric_shapes
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
)

## Node executables
add_executable(test_node   src/test_node.cpp)
add_executable(node_A      src/node_A.cpp)
add_executable(node_B      src/node_B.cpp)
add_executable(node_C      src/node_C.cpp)
add_executable(tables_node     src/tables_node.cpp)

## Link libraries
target_link_libraries(test_node   ${catkin_LIBRARIES} ${geometric_shapes_LIBRARIES})
target_link_libraries(node_A      ${catkin_LIBRARIES} ${geometric_shapes_LIBRARIES})
target_link_libraries(node_B      ${catkin_LIBRARIES} ${geometric_shapes_LIBRARIES})
target_link_libraries(node_C      ${catkin_LIBRARIES} ${geometric_shapes_LIBRARIES})
target_link_libraries(tables_node     ${catkin_LIBRARIES} ${geometric_shapes_LIBRARIES})

## Ensure messages are built before the nodes
add_dependencies(test_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_A     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_B     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_C     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tables_node    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
