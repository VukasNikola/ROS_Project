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

