#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Helper function to move the head with interruption check
void moveHead(ros::Publisher& head_pub, double yaw, double pitch, double duration, const std::string& description) {
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = {"head_1_joint", "head_2_joint"};

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {yaw, pitch};  // Yaw and Pitch
  point.time_from_start = ros::Duration(duration);

  traj.points.push_back(point);

  ROS_INFO("Moving head to %s [yaw: %.2f, pitch: %.2f]", description.c_str(), yaw, pitch);
  head_pub.publish(traj);

  // Use ros::Rate instead of sleep to allow Ctrl+C to interrupt
  ros::Rate rate(10);  // 10 Hz loop
  double elapsed = 0.0;
  while (ros::ok() && elapsed < (duration + 1.0)) {
    rate.sleep();
    elapsed += 0.1;
  }
}

// Service callback to move the head down and then back up
bool moveHeadDownAndUp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  ros::NodeHandle nh;
  ros::Publisher head_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);

  ros::Rate wait_rate(10);
  double elapsed = 0.0;
  while (ros::ok() && elapsed < 1.0) {  // Ensure the publisher is connected
    wait_rate.sleep();
    elapsed += 0.1;
  }

  if (!ros::ok()) return false;  // Stop if interrupted

  // 1️ Move the head down
  moveHead(head_pub, 0.0, -1.3, 2.0, "look down");

  // 2️ Pause for 2 seconds while looking down
  elapsed = 0.0;
  while (ros::ok() && elapsed < 2.0) {
    wait_rate.sleep();
    elapsed += 0.1;
  }

  if (!ros::ok()) return false;

  // 3️ Move the head back up
  moveHead(head_pub, 0.0, 0.0, 2.0, "look forward");

  res.success = true;
  res.message = "Head moved down and returned to forward position.";
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_head_down_node");
  ros::NodeHandle nh;

  // Advertise the service
  ros::ServiceServer service = nh.advertiseService("/move_head_down", moveHeadDownAndUp);
  ROS_INFO("Head down and up service is ready.");

  // ros::spin() will now properly handle Ctrl+C
  ros::spin();

  ROS_INFO("Shutting down move_head_down_node.");
  return 0;
}
