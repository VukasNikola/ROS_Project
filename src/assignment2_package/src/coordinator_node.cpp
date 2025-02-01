#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>  // For M_PI

// Straight-line service (provided by tiago_iaslab_simulation)
#include <tiago_iaslab_simulation/Coeffs.h>
// Custom place service from your package (PlaceObject.srv)
#include <assignment2_package/PlaceObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinator_node");
  ros::NodeHandle nh;

  // Create an action client for move_base.
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ROS_INFO("Waiting for move_base action server...");
  ac.waitForServer();
  ROS_INFO("Connected to move_base action server.");

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  // --- Navigation Sequence to the Pick-Up Table ---
  // 1st Goal: (8.9, -1) with yaw = 0.
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
  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to reach first goal.");
    return 1;
  }
  ROS_INFO("Reached first goal.");

  // 2nd Goal: (8.9, -2.5) with yaw = 0.
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.9;
  goal.target_pose.pose.position.y = -2.5;
  {
    tf2::Quaternion q;
    double yaw = 0.0;
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }
  ROS_INFO("Sending 2nd goal: (8.9, -2.5) with yaw = 0");
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to reach second goal.");
    return 1;
  }
  ROS_INFO("Reached second goal.");

  // 3rd Goal: Adjust to (8.7, -2.5) with yaw = π (robot faces negative x).
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.7;
  goal.target_pose.pose.position.y = -2.5;
  {
    tf2::Quaternion q;
    double yaw = M_PI;
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }
  ROS_INFO("Sending 3rd goal: (8.7, -2.5) with yaw = π");
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to reach third goal.");
    return 1;
  }
  ROS_INFO("Successfully reached the pick-up table.");

  ros::Duration(2.0).sleep();  // Allow time for object detection and pick-up.

  // --- Navigation to the Placing Table ---
  // Assuming the placing table is to the right of the current location,
  // shift 0.5 meter to the right.
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.7;  // same x
  goal.target_pose.pose.position.y = -2.5 + 0.5;  // shift right by 0.5 m
  {
    tf2::Quaternion q;
    // Maintain the same orientation (facing negative x)
    double yaw = M_PI;
    q.setRPY(0.0, 0.0, yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }
  ROS_INFO("Navigating to the placing table...");
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to reach the placing table.");
    return 1;
  }
  ROS_INFO("Reached the placing table.");

  // --- Retrieve Straight Line Coefficients ---
  ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
  ROS_INFO("Waiting for /straight_line_srv...");
  coeffs_client.waitForExistence();
  tiago_iaslab_simulation::Coeffs coeffs_srv;
  coeffs_srv.request.ready = true;
  if (!coeffs_client.call(coeffs_srv)) {
    ROS_ERROR("Failed to call /straight_line_srv.");
    return 1;
  }
  double m_coeff = coeffs_srv.response.coeffs[0];
  double q_coeff = coeffs_srv.response.coeffs[1];
  ROS_INFO("Received line coefficients: m = %f, q = %f", m_coeff, q_coeff);

  // --- Compute the Placing Pose ---
  // Choose an x coordinate (in the placing table frame) and compute y = m*x + q.
  double x_target = 0.5;
  double y_target = m_coeff * x_target + q_coeff;
  geometry_msgs::PoseStamped placing_pose_table;
  // Use the table’s frame (e.g., if static, you can use "tag_10" or a fixed table frame)
  placing_pose_table.header.frame_id = "tag_10";
  placing_pose_table.header.stamp = ros::Time::now();
  placing_pose_table.pose.position.x = x_target;
  placing_pose_table.pose.position.y = y_target;
  placing_pose_table.pose.position.z = 0.1;  // 10 cm above table
  placing_pose_table.pose.orientation.w = 1.0;

  // Transform the placing pose from the table frame to "base_link"
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::PoseStamped placing_pose_robot;
  ros::Time start_time = ros::Time::now();
  while (!tfBuffer.canTransform("base_link", placing_pose_table.header.frame_id, ros::Time(0), ros::Duration(1.0))) {
    if ((ros::Time::now() - start_time).toSec() > 10.0) {
      ROS_ERROR("Transform from %s to base_link not available after 10 seconds", placing_pose_table.header.frame_id.c_str());
      return 1;
    }
    ROS_WARN("Waiting for transform from %s to base_link...", placing_pose_table.header.frame_id.c_str());
    ros::Duration(0.5).sleep();
  }
  try {
    geometry_msgs::TransformStamped transformStamped =
      tfBuffer.lookupTransform("base_link", placing_pose_table.header.frame_id, ros::Time(0), ros::Duration(3.0));
    tf2::doTransform(placing_pose_table, placing_pose_robot, transformStamped);
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("TF transform error: %s", ex.what());
    return 1;
  }
  ROS_INFO("Placing pose in robot frame: (%.2f, %.2f, %.2f)",
           placing_pose_robot.pose.position.x,
           placing_pose_robot.pose.position.y,
           placing_pose_robot.pose.position.z);

  // --- Request the Place Operation from the Manipulation Node ---
  ros::ServiceClient place_client = nh.serviceClient<assignment2_package::PlaceObject>("place_object");
  ROS_INFO("Waiting for service place_object...");
  place_client.waitForExistence();
  assignment2_package::PlaceObject place_srv;
  place_srv.request.target_pose = placing_pose_robot;
  if (place_client.call(place_srv)) {
    if (place_srv.response.success)
      ROS_INFO("Place operation succeeded: %s", place_srv.response.message.c_str());
    else
      ROS_ERROR("Place operation failed: %s", place_srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call place_object service.");
  }

  return 0;
}
