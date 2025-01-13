cmake_minimum_required(VERSION 3.0.2)
project(tiago_movement)

## Compile as C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp)

target_link_libraries(cmd_vel_subscriber
  ${catkin_LIBRARIES}
)

add_dependencies(cmd_vel_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
