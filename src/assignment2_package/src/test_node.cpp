#include "gripper_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_A");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  gripper_control::GripperController gc("gripper");
  gc.registerNamedTarget("open", 0.44);
  gc.registerNamedTarget("closed", 0.0);
  gc.close(5);
  // gc.open();

  ros::shutdown();
  return 0;
}
