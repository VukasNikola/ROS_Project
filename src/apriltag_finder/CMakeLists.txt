cmake_minimum_required(VERSION 3.0.2)
project(apriltag_finder)

## Compile as C++14, supported in ROS Noetic
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  actionlib
  move_base_msgs
  message_generation
  apriltag_ros  # Ensure this is included if you depend on apriltag_ros messages
)
add_service_files(
  FILES
  ApriltagsIDsSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf actionlib move_base_msgs message_runtime apriltag_ros
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(node_a src/node_a.cpp)
add_executable(node_b src/node_b.cpp)
add_executable(ids_generator_node src/ids_generator_node.cpp)  # Add this line

## Add dependencies to executables
add_dependencies(node_a ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_b ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ids_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  # Add this line

## Link libraries
target_link_libraries(node_a
  ${catkin_LIBRARIES}
)

target_link_libraries(node_b
  ${catkin_LIBRARIES}
)

target_link_libraries(ids_generator_node  # Add this block
  ${catkin_LIBRARIES}
)
