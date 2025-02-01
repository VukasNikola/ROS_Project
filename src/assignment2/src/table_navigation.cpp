#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>  // For M_PI

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_nav_node");
  ros::NodeHandle nh;

  // Create an action client that communicates with move_base
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ROS_INFO("Waiting for the move_base action server...");
  ac.waitForServer();
  ROS_INFO("Connected to the move_base action server.");

  // -----------------------------
  // First Goal: Move to (8.9, -1) with yaw = 0.
  // -----------------------------
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  
  // Set the first goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.9;
  goal.target_pose.pose.position.y = -1.0;
  {
    tf2::Quaternion q;
    double yaw = 0.0;  // facing positive x
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }

  ROS_INFO("Sending 1st goal: (8.9, -1) with yaw = 0");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      ROS_INFO("Successfully reached the first goal!");
  }
  else
  {
      ROS_ERROR("The robot failed to reach the first goal.");
      return 1;
  }

  // -----------------------------
  // Second Goal: Move to (8.9, -2.5) with yaw = 0.
  // -----------------------------
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.9;
  goal.target_pose.pose.position.y = -2.5;
  {
    tf2::Quaternion q;
    double yaw = 0.0;  // still facing positive x
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }

  ROS_INFO("Sending 2nd goal: (8.9, -2.5) with yaw = 0");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      ROS_INFO("Successfully reached the second goal!");
  }
  else
  {
      ROS_ERROR("The robot failed to reach the second goal.");
      return 1;
  }

  // -----------------------------
  // Third Goal: Adjust position to (8.8, -2.5) and rotate in place to face negative x (yaw = π).
  // -----------------------------
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.7;  // slight adjustment in x
  goal.target_pose.pose.position.y = -2.5; // same y as second goal
  {
    tf2::Quaternion q;
    double yaw = M_PI;  // π radians (i.e., 180°): robot faces negative x direction.
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }

  ROS_INFO("Sending 3rd goal: (8.8, -2.5)");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      ROS_INFO("Successfully reached the final goal with correct orientation!");
  }
  else
  {
      ROS_ERROR("The robot failed to reach the final goal.");
      return 1;
  }

  return 0;
}
