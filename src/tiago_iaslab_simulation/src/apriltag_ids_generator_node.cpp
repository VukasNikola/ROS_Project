#include <ros/ros.h>
#include <tiago_iaslab_simulation/ApriltagIds.h>
#include <vector>
#include <ctime>

int main(int argc, char **argv){
  ros::init(argc, argv, "ids_generator_node");
  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  ApriltagIds ids(nh_ptr);

  ros::spin();

  return 0;
}