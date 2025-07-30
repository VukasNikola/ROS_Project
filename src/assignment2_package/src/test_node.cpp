#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_open_close_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 1) Connect to the "gripper" planning group
  moveit::planning_interface::MoveGroupInterface gripper("gripper");

  // 2) Specify your desired joint positions for “open”
  std::map<std::string, double> open_position;
  open_position["gripper_left_finger_joint"]  =  0.04;  // adjust to your robot’s URDF values
  open_position["gripper_right_finger_joint"] =  0.04;

  gripper.setJointValueTarget(open_position);

  // 3) Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan open_plan;
  bool success = (gripper.plan(open_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
              && (gripper.execute(open_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
    ROS_INFO("Gripper opened!");
  else
    ROS_ERROR("Failed to open gripper.");

  // 4) (Optional) now close
  std::map<std::string, double> closed_position;
  closed_position["gripper_left_finger_joint"]  =  0.015;
  closed_position["gripper_right_finger_joint"] =  0.015;

  gripper.setJointValueTarget(closed_position);
  moveit::planning_interface::MoveGroupInterface::Plan close_plan;
  success = (gripper.plan(close_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
         && (gripper.execute(close_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
    ROS_INFO("Gripper closed!");
  else
    ROS_ERROR("Failed to close gripper.");

  ros::shutdown();
  return 0;
}
