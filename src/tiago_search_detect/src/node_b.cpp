#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sstream>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>

// Global variables for target IDs.
std::vector<int> g_target_ids;
bool g_target_ids_received = false;

// Global map to store the (first) refined detection for each target tag (keyed by tag id).
std::map<int, geometry_msgs::PoseStamped> g_finalDetections;

// Global variable to record the time of the last detection update.
ros::Time g_lastDetectionTime;

// Timeout (in seconds) after the last detection at which we consider the search finished.
const double DETECTION_FINISH_TIMEOUT = 2.0;

// Global flag to indicate that detections have been finalized (and no further updates are accepted).
bool g_detection_finalized = false;

// Global pointers for TF2.
tf2_ros::Buffer* g_tfBuffer = nullptr;
tf2_ros::TransformListener* g_tfListener = nullptr;

// Global publishers for feedback and final cube positions.
ros::Publisher g_feedback_pub;
ros::Publisher g_cube_positions_pub;

// --- Callback for receiving target IDs from Node A ---
void targetIDsCallback(const std_msgs::Int32MultiArray::ConstPtr &msg) {
  g_target_ids = msg->data;
  g_target_ids_received = true;
  //oss
  std::ostringstream oss;
  oss << "Received target IDs: ";
  for (const int id : g_target_ids) {
    oss << id << " ";
  }
  std_msgs::String feedback_msg;
  feedback_msg.data = oss.str();
  g_feedback_pub.publish(feedback_msg);
  ROS_INFO_STREAM("[Node B] " << feedback_msg.data);
}

// --- Callback for AprilTag detections ---
void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  // If detections have already been finalized, ignore further detections.
  if (g_detection_finalized)
    return;

  if (!g_target_ids_received) {
    ROS_WARN_THROTTLE(5, "[Node B] Waiting for target IDs...");
    return;
  }
  
  // Update the time of the last detection.
  g_lastDetectionTime = ros::Time::now();
  
  // Publish scanning feedback.
  {
    std_msgs::String fb_msg;
    fb_msg.data = "Robot is scanning for AprilTags.";
    g_feedback_pub.publish(fb_msg);
  }

  // Process each detection.
  for (size_t i = 0; i < msg->detections.size(); ++i) {
    // Assume the first element in the id vector is the detection's ID.
    int detected_id = msg->detections[i].id[0];
    
    // Only process if this tag is in our target list.
    if (std::find(g_target_ids.begin(), g_target_ids.end(), detected_id) == g_target_ids.end())
      continue;
    
    // Check if we already detected this tag.
    if (g_finalDetections.find(detected_id) != g_finalDetections.end()) {
      // Already detected and stored—ignore subsequent detections.
      continue;
    }
    
    std::ostringstream oss;
    oss << "Saw tag with ID: " << detected_id;
    
    // Get the pose in the camera frame from the detection.
    const geometry_msgs::Pose &tag_pose = msg->detections[i].pose.pose.pose;
    oss << ", Position in camera frame: (" 
        << tag_pose.position.x << ", " 
        << tag_pose.position.y << ", " 
        << tag_pose.position.z << ")";
    
    // Construct input PoseStamped.
    geometry_msgs::PoseStamped pose_in;
    pose_in.header = msg->header;  // Assumes the header's frame_id is the camera frame.
    pose_in.pose = tag_pose;

    // Prepare output PoseStamped.
    geometry_msgs::PoseStamped pose_out;
    try {
      // Lookup the transformation from the camera frame to the "map" frame.
      geometry_msgs::TransformStamped transformStamped = g_tfBuffer->lookupTransform("map", pose_in.header.frame_id, ros::Time(0), ros::Duration(1.0));
      // Transform pose_in into pose_out (now in the map frame).
      tf2::doTransform(pose_in, pose_out, transformStamped);
      
      oss << " --> MATCHES a target ID! Transformed pose in map frame: ("
          << pose_out.pose.position.x << ", " 
          << pose_out.pose.position.y << ", " 
          << pose_out.pose.position.z << ")";
      
      // Save this detection for the tag, if not already stored.
      g_finalDetections[detected_id] = pose_out;
      
      // Publish feedback for this detection update.
      std_msgs::String fb_msg;
      std::ostringstream updateMsg;
      updateMsg << "Set detection for tag " << detected_id << ": Transformed pose ("
                << pose_out.pose.position.x << ", "
                << pose_out.pose.position.y << ", "
                << pose_out.pose.position.z << ") in map frame.";
      fb_msg.data = updateMsg.str();
      g_feedback_pub.publish(fb_msg);
      ROS_INFO_STREAM("[Node B] " << fb_msg.data);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR_STREAM("[Node B] TF transform error: " << ex.what());
      continue;
    }
    ROS_DEBUG_STREAM("[Node B] " << oss.str());
  }
}

// --- Timer callback: publish final cube positions when detection has finished ---
void publishFinalCubesCallback(const ros::TimerEvent&) {
  // Check if no new detection has been received for longer than the timeout,
  // and we have at least one detection.
  if (!g_detection_finalized &&
      (ros::Time::now() - g_lastDetectionTime).toSec() > DETECTION_FINISH_TIMEOUT &&
      !g_finalDetections.empty())
  {
    geometry_msgs::PoseArray cubePositions;
    cubePositions.header.stamp = ros::Time::now();
    cubePositions.header.frame_id = "map";
    
    std::ostringstream oss;
    oss << "Detection finished. Publishing final cube positions. ";
    
    // Add each stored pose to the PoseArray.
    for (const auto &entry : g_finalDetections) {
      cubePositions.poses.push_back(entry.second.pose);
      oss << "Tag " << entry.first << " at ("
          << entry.second.pose.position.x << ", "
          << entry.second.pose.position.y << ", "
          << entry.second.pose.position.z << "); ";
    }
    
    // Publish the final positions.
    g_cube_positions_pub.publish(cubePositions);
    
    // Publish a final feedback message.
    std_msgs::String fb_msg;
    fb_msg.data = oss.str();
    g_feedback_pub.publish(fb_msg);
    ROS_INFO_STREAM("[Node B] " << fb_msg.data);
    
    // Mark detection as finalized so further detections are ignored.
    g_detection_finalized = true;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_b");
  ros::NodeHandle nh;
  
  // Setup global publishers.
  g_feedback_pub = nh.advertise<std_msgs::String>("robot_feedback", 10);
  g_cube_positions_pub = nh.advertise<geometry_msgs::PoseArray>("cube_positions", 10);
  
  // Subscribers.
  ros::Subscriber target_ids_sub = nh.subscribe("target_ids", 10, targetIDsCallback);
  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagDetectionsCallback);
  
  // Setup TF2: create a Buffer and a TransformListener.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  g_tfBuffer = &tfBuffer;
  
  // Initialize the last detection time.
  g_lastDetectionTime = ros::Time::now();
  
  // Timer callback: periodically check if no new detections have come in to publish the final positions.
  ros::Timer timer = nh.createTimer(ros::Duration(5.0), publishFinalCubesCallback);
  
  ros::spin();
  return 0;
}
