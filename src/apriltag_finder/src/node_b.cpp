// node_b.cpp

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "apriltag_finder/ApriltagsIDsSrv.h" // Correct service include
#include <vector>
#include <string>
#include <sstream>
#include <mutex>

// Include necessary TF headers
#include <tf/transform_listener.h>

// Include actionlib for move_base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// Include Apriltag detection message
// Replace with the actual message type used by your apriltag detection node
#include <apriltag_ros/AprilTagDetectionArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NodeB
{
public:
    NodeB()
        : move_base_client_("move_base", true)
    {
        // Initialize publishers and subscribers
        selected_ids_sub_ = nh_.subscribe("/selected_apriltag_ids", 10, &NodeB::idsCallback, this);
        feedback_pub_ = nh_.advertise<std_msgs::String>("/node_b/feedback", 10);
        cube_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("/node_b/cube_positions", 10);

        // Initialize TF listener
        // TF listener is initialized in the constructor

        // Subscribe to apriltag detections
        apriltag_sub_ = nh_.subscribe("/apriltag_detections", 10, &NodeB::apriltagCallback, this);

        // Wait for move_base action server
        ROS_INFO("Node B: Waiting for move_base action server...");
        move_base_client_.waitForServer();
        ROS_INFO("Node B: Connected to move_base action server.");
    }

    void idsCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (selected_ids_received_)
        {
            ROS_WARN("Node B: Selected IDs already received. Ignoring new IDs.");
            return;
        }

        // Parse the received IDs
        selected_ids_ = parseIDs(msg->data);
        ROS_INFO("Node B: Received selected IDs: %s", vectorToString(selected_ids_).c_str());

        // Provide feedback
        std_msgs::String feedback_msg;
        feedback_msg.data = "Received selected apriltag IDs. Starting navigation.";
        feedback_pub_.publish(feedback_msg);

        // Start navigation
        navigateWaypoints();

        selected_ids_received_ = true;
    }

    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& detection : msg->detections)
        {
            // Handle multiple IDs per detection if applicable
            for (const auto& tag_id : detection.id) // Assuming detection.id is a vector<int>
            {
                if (std::find(selected_ids_.begin(), selected_ids_.end(), tag_id) != selected_ids_.end())
                {
                    // Avoid processing the same tag multiple times
                    if (std::find(found_ids_.begin(), found_ids_.end(), tag_id) != found_ids_.end())
                    {
                        continue;
                    }

                    ROS_INFO("Node B: Found selected apriltag ID: %d", tag_id);

                    // Provide feedback
                    std_msgs::String feedback_msg;
                    std::stringstream ss;
                    ss << "Found apriltag ID: " << tag_id;
                    feedback_msg.data = ss.str();
                    feedback_pub_.publish(feedback_msg);

                    // Get pose relative to camera
                    geometry_msgs::PoseWithCovarianceStamped pose_with_cov = detection.pose;

                    // Convert PoseWithCovarianceStamped to PoseStamped
                    geometry_msgs::PoseStamped pose_cam;
                    pose_cam.header = pose_with_cov.header;
                    pose_cam.pose = pose_with_cov.pose.pose;

                    // Transform to map frame
                    geometry_msgs::PoseStamped pose_map;
                    try
                    {
                        tf_listener_.transformPose("map", pose_cam, pose_map);
                        cube_pos_pub_.publish(pose_map.pose);
                        ROS_INFO("Node B: Published cube position in map frame.");

                        // Add to found IDs
                        found_ids_.push_back(tag_id);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("Node B: TF transformation failed: %s", ex.what());
                    }

                    // Check if all selected IDs have been found
                    if (found_ids_.size() == selected_ids_.size())
                    {
                        std_msgs::String finish_msg;
                        finish_msg.data = "Detection finished. All selected apriltags found.";
                        feedback_pub_.publish(finish_msg);
                        ROS_INFO("Node B: All selected apriltags have been found.");
                        // Optionally, cancel any ongoing navigation
                        move_base_client_.cancelAllGoals();
                        return;
                    }
                }
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber selected_ids_sub_;
    ros::Publisher feedback_pub_;
    ros::Publisher cube_pos_pub_;
    ros::Subscriber apriltag_sub_;

    MoveBaseClient move_base_client_;
    tf::TransformListener tf_listener_;

    std::vector<int> selected_ids_;
    std::vector<int> found_ids_;
    bool selected_ids_received_ = false;

    std::mutex mutex_;

    std::vector<int> parseIDs(const std::string& ids_str)
    {
        std::vector<int> ids;
        std::stringstream ss(ids_str);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            try
            {
                int id = std::stoi(item);
                ids.push_back(id);
            }
            catch (const std::invalid_argument& e)
            {
                ROS_WARN("Node B: Invalid ID encountered: %s", item.c_str());
            }
        }
        return ids;
    }

    std::string vectorToString(const std::vector<int>& vec, const std::string& delimiter = " ")
    {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i != vec.size() - 1)
                ss << delimiter;
        }
        return ss.str();
    }

    void navigateWaypoints()
    {
        // Define navigation goals or implement exploration strategy
        // For simplicity, let's assume we have predefined waypoints
        std::vector<std::pair<float, float>> waypoints = getWaypoints();

        for (const auto& point : waypoints)
        {
            if (found_ids_.size() == selected_ids_.size())
            {
                ROS_INFO("Node B: All selected apriltags found. Stopping navigation.");
                break;
            }

            move_base_msgs::MoveBaseGoal goal;

            // Set frame ID and timestamp
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Set position
            goal.target_pose.pose.position.x = point.first;
            goal.target_pose.pose.position.y = point.second;
            goal.target_pose.pose.position.z = 0.0;

            // Set orientation (facing forward)
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Node B: Navigating to waypoint: [x: %f, y: %f]", point.first, point.second);

            // Provide feedback
            std_msgs::String feedback_msg;
            std::stringstream ss;
            ss << "Moving to waypoint: [" << point.first << ", " << point.second << "]";
            feedback_msg.data = ss.str();
            feedback_pub_.publish(feedback_msg);

            // Send goal
            move_base_client_.sendGoal(goal);

            // Wait for the result
            bool finished_before_timeout = move_base_client_.waitForResult(ros::Duration(60.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = move_base_client_.getState();
                ROS_INFO("Node B: Move base action finished: %s", state.toString().c_str());

                // Provide feedback
                std_msgs::String feedback_msg_reached;
                feedback_msg_reached.data = "Reached waypoint: [" + std::to_string(point.first) + ", " + std::to_string(point.second) + "]";
                feedback_pub_.publish(feedback_msg_reached);
            }
            else
            {
                ROS_WARN("Node B: Move base action did not finish before the timeout.");
                // Optionally, handle timeout (e.g., retry or move to next waypoint)
            }
        }

        // After navigating all waypoints
        std_msgs::String finish_msg;
        finish_msg.data = "Navigation completed.";
        feedback_pub_.publish(finish_msg);
        ROS_INFO("Node B: Navigation completed.");
    }

    std::vector<std::pair<float, float>> getWaypoints()
    {
        // Define your waypoints here. This could be replaced with an exploration algorithm.
        return {
            {1.0, 0.0},
            {1.0, 1.0},
            {0.0, 1.0},
            {-1.0, 1.0},
            {-1.0, 0.0},
            {-1.0, -1.0},
            {0.0, -1.0},
            {1.0, -1.0},
            {1.0, 0.0}
        };
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    NodeB node_b;
    ros::spin();
    return 0;
}
