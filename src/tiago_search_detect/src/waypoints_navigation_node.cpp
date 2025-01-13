#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

struct Goal2D {
  double x;
  double y;
  double yaw;
};

class WaypointsNavigator {
public:
  WaypointsNavigator(ros::NodeHandle& nh)
    : nh_(nh),
      ac_("move_base", true)  // Connect to move_base
  {
    // 1) Wait for move_base server.
    ROS_INFO("Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("Connected to move_base action server.");

    // 2) Load the waypoints file parameter.
    std::string waypoints_file;
    nh_.param<std::string>("waypoints_file", waypoints_file, "");
    if (waypoints_file.empty()) {
      ROS_ERROR("No waypoints file provided! Set the 'waypoints_file' parameter.");
      ros::shutdown();
      return;
    }

    // 3) Load waypoints from YAML.
    if (!loadWaypoints(waypoints_file)) {
      ROS_ERROR("Failed to load waypoints.");
      ros::shutdown();
      return;
    }

    // 4) Start the waypoint routine.
    visitWaypoints();
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  std::vector<Goal2D> waypoints_;

  bool loadWaypoints(const std::string& filepath) {
    try {
      YAML::Node config = YAML::LoadFile(filepath);
      if (!config["waypoints"]) {
        ROS_ERROR("YAML missing 'waypoints' key.");
        return false;
      }
      for (auto w : config["waypoints"]) {
        Goal2D goal;
        goal.x = w["x"].as<double>();
        goal.y = w["y"].as<double>();
        goal.yaw = w["yaw"] ? w["yaw"].as<double>() : 0.0;
        waypoints_.push_back(goal);
      }
      ROS_INFO("Loaded %lu waypoints.", waypoints_.size());
      return true;
    } catch (std::exception &e) {
      ROS_ERROR("YAML parse error: %s", e.what());
      return false;
    }
  }

  // Service call to move the head down and back up
  void callMoveHeadDownService() {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/move_head_down");

    ROS_INFO("Waiting for /move_head_down service to be available...");
    if (!client.waitForExistence(ros::Duration(5.0))) {
      ROS_ERROR("Service /move_head_down is not available.");
      return;
    }

    ROS_INFO("Calling /move_head_down service...");
    std_srvs::Trigger srv;
    if (client.call(srv)) {
      if (srv.response.success) {
        ROS_INFO("Service call succeeded: %s", srv.response.message.c_str());
      } else {
        ROS_WARN("Service call failed: %s", srv.response.message.c_str());
      }
    } else {
      ROS_ERROR("Failed to call /move_head_down service.");
    }
  }

  void visitWaypoints() {
    for (size_t i = 0; i < waypoints_.size() && ros::ok(); ++i) {
      ROS_INFO("Navigating to waypoint %lu: (x=%.2f, y=%.2f, yaw=%.2f)",
               i + 1, waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = waypoints_[i].x;
      goal.target_pose.pose.position.y = waypoints_[i].y;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, waypoints_[i].yaw);
      goal.target_pose.pose.orientation = tf2::toMsg(q);

      ac_.sendGoal(goal);

      // Wait for the navigation result but allow Ctrl+C interruption
      while (ros::ok() && !ac_.waitForResult(ros::Duration(0.5))) {
        // Check if still running
      }

      if (!ros::ok()) {
        ROS_WARN("Shutdown detected. Stopping waypoint navigation.");
        break;
      }

      if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached waypoint %lu.", i + 1);

        // Move the head down and back up
        callMoveHeadDownService();

        ros::Duration(1.0).sleep();  // Optional pause before next waypoint
      } else {
        ROS_WARN("Failed to reach waypoint %lu.", i + 1);
      }
    }
    ROS_INFO("Finished visiting all waypoints.");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoints_navigation_node");
  ros::NodeHandle private_nh("~");

  WaypointsNavigator navigator(private_nh);

  ros::spin();
  return 0;
}
