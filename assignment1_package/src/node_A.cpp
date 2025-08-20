#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>

// Use your provided service header (adjust the path if needed)
#include </home/nikolavukas/project/src/tiago_iaslab_simulation/include/tiago_iaslab_simulation/ApriltagIds.h> // BAD NEEDS FIXING

// Global publisher to send target IDs to Node B.
ros::Publisher target_ids_pub;

// Callback to receive feedback from Node B.
void feedbackCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO_STREAM("[Node A] Feedback: " << msg->data);
}

// Callback to receive the final cube positions from Node B.
void cubePositionsCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
  ROS_INFO("[Node A] Received final cube positions (in the map frame):");
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    ROS_INFO_STREAM("Cube " << i << " Position: ["
                    << msg->poses[i].position.x << ", "
                    << msg->poses[i].position.y << ", "
                    << msg->poses[i].position.z << "]");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "node_a");
  ros::NodeHandle nh;

  // Create a service client to call the AprilTag IDs service.
  ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");

  ROS_INFO("Waiting for service /apriltag_ids_srv...");
  client.waitForExistence();

  tiago_iaslab_simulation::Objs srv;
  srv.request.ready = true; // Indicate that we are ready

  if (client.call(srv)) {
    ROS_INFO("Service call successful; received AprilTag IDs.");
    
    // Print each received ID
    std::ostringstream oss;
    oss << "Received AprilTag IDs: ";
    for (const int id : srv.response.ids) {
      oss << id << " ";
    }
    ROS_INFO_STREAM("[Node A] " << oss.str());
    
    // Publish the target IDs to Node B.
    std_msgs::Int32MultiArray target_ids_msg;
    for (const int id : srv.response.ids) {
      target_ids_msg.data.push_back(id);
    }
    
    // Create the publisher with latch=true so that new subscribers receive the message.
    target_ids_pub = nh.advertise<std_msgs::Int32MultiArray>("target_ids", 10, true);
    ros::Duration(0.5).sleep();  // Give time for the subscriber to connect.
    
    target_ids_pub.publish(target_ids_msg);
    ROS_INFO_STREAM("[Node A] Published target IDs to the topic 'target_ids'.");
  } else {
    ROS_ERROR("Failed to call service /apriltag_ids_srv.");
    return 1;
  }

  // Subscribers for feedback and final cube positions.
  ros::Subscriber feedback_sub = nh.subscribe("robot_feedback", 10, feedbackCallback);
  ros::Subscriber cube_positions_sub = nh.subscribe("cube_positions", 10, cubePositionsCallback);

  ros::spin();
  return 0;
}
