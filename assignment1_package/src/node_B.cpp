/**
 * Node B - Navigation Action Server
 * 
 * This node implements a comprehensive navigation and detection system for TIAGo.
 * It serves as an action server that receives navigation tasks from Node A and executes
 * autonomous navigation through predefined waypoints while continuously detecting AprilTags.
 * 
 * Key Features:
 * - Autonomous navigation through hardcoded waypoints
 * - Continuous AprilTag detection during movement and at waypoints
 * - 360-degree rotation scanning at specific waypoints
 * - Head control for optimal detection angles
 * - Thread-safe detection handling
 * - Real-time feedback to action client
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <assignment1_package/NavigationTaskAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <set>

// Type aliases for cleaner code readability
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<assignment1_package::NavigationTaskAction> NavigationServer;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadClient;

/**
 * NavigationActionServer Class
 * 
 * Main class that implements the navigation action server functionality.
 * Handles waypoint navigation, AprilTag detection, and robot control.
 */
class NavigationActionServer
{
private:
    // Core ROS Components
    ros::NodeHandle nh_;                    // Node handle for ROS communication
    NavigationServer action_server_;        // Action server to communicate with Node A
    MoveBaseClient move_base_client_;       // Client to control robot navigation
    HeadClient head_client_;               // Client to control robot head movement
    ros::Subscriber apriltag_sub_;         // Subscriber for AprilTag detections

    // Transform and Coordinate Systems
    tf2_ros::Buffer tf_buffer_;            // Buffer for coordinate transformations
    tf2_ros::TransformListener tf_listener_; // Listener for coordinate frame updates

    // Navigation Data
    std::vector<std::pair<double, double>> waypoints_; // Predefined waypoints (x, y coordinates)
    std::vector<int32_t> target_ids_;      // AprilTag IDs we need to find (from Node A)
    std::map<int, geometry_msgs::PoseStamped> detected_cubes_; // Found cubes: ID -> 3D position
    std::mutex detected_cubes_mutex_;      // Thread safety for concurrent detection access

    // Action Communication
    assignment1_package::NavigationTaskFeedback feedback_; // Feedback message for Node A
    assignment1_package::NavigationTaskResult result_;     // Final result message for Node A

    // State Management
    bool navigation_active_;               // Flag to control when detection processing is active
    
    // Configuration: Waypoints that don't require 360-degree rotation (1-indexed)
    // These waypoints are positioned such that a full rotation isn't necessary for detection
    std::set<int> no_rotation_waypoints_;

public:
    /**
     * Constructor
     * Initializes all components, sets up action server, and prepares robot for navigation
     */
    NavigationActionServer() : action_server_(nh_, "navigation_task", boost::bind(&NavigationActionServer::executeNavigation, this, _1), false),
                               move_base_client_("move_base", true),
                               head_client_("/head_controller/follow_joint_trajectory", true),
                               tf_listener_(tf_buffer_),
                               navigation_active_(false)
    {
        // Configure waypoints that don't need rotation (strategic positions)
        no_rotation_waypoints_ = {1, 3, 6, 7, 9};

        // Load predefined navigation waypoints
        loadWaypoints();

        // Set up continuous AprilTag detection
        // This subscriber runs throughout the robot's operation
        apriltag_sub_ = nh_.subscribe("/tag_detections", 1,
                                      &NavigationActionServer::apriltagCallback, this);

        // Wait for move_base navigation system to be ready
        ROS_INFO("[Node B] Waiting for move_base action server...");
        move_base_client_.waitForServer();

        // Wait for head controller to be ready
        ROS_INFO("[Node B] Waiting for head controller...");
        head_client_.waitForServer();

        // Position head for optimal detection at startup
        moveHeadDown();

        // Start the action server to accept goals from Node A
        action_server_.start();
        ROS_INFO("[Node B] Navigation action server started!");
    }

    /**
     * Load Predefined Waypoints
     * Hardcoded waypoints that provide optimal coverage of the environment
     * These coordinates are in the map frame
     */
    void loadWaypoints()
    {
        // Strategic waypoints covering the environment for comprehensive AprilTag detection
        waypoints_ = {
            {-0.09, -1.01}, {7.18, -0.77}, {8.58, -2.37}, {8, -3.37}, {12.15, -2.87}, 
            {13.08, -1.37}, {12.08, 1.13}, {10.70, 0.70}, {8.78, 0.63}
        };
        ROS_INFO("[Node B] Loaded %zu hardcoded waypoints", waypoints_.size());
    }

    /**
     * Move Head to Detection Position
     * Positions the robot's head at an optimal angle for AprilTag detection
     * Tilts head downward to better detect cubes on the ground/tables
     */
    void moveHeadDown()
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.0, -1.3}; // Pan=0 (straight ahead), Tilt=-1.3 (downward)
        point.time_from_start = ros::Duration(2.0);
        goal.trajectory.points.push_back(point);

        head_client_.sendGoal(goal);
        head_client_.waitForResult();
        ROS_INFO("[Node B] Head moved down for cube detection");
    }

    /**
     * Perform 360-Degree Rotation Scan
     * Rotates the robot in place to scan for AprilTags in all directions
     * Used at waypoints where comprehensive scanning is needed
     * 
     * @param position Current waypoint position (for reference)
     * @param waypoint_num Waypoint number for logging and feedback
     */
    void rotateInPlace(const std::pair<double, double> &position, int waypoint_num)
    {
        // Create velocity publisher for direct robot base control
        ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        ros::Duration(0.1).sleep(); // Brief wait for publisher to establish connection

        // Rotation parameters - tuned for smooth, comprehensive scanning
        double angular_velocity = 0.5;  // rad/s (moderate speed for stable detection)
        double target_angle = 2 * M_PI; // Full 360-degree rotation

        // Calculate time needed for complete rotation
        double rotation_time = target_angle / angular_velocity;

        // Create rotation command (counter-clockwise)
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.0;   // No linear movement
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;  // No roll
        twist_msg.angular.y = 0.0;  // No pitch
        twist_msg.angular.z = angular_velocity; // Yaw rotation (positive = counter-clockwise)

        // Update feedback with scanning status and current detection count
        std::string scan_msg = "Scanning waypoint " + std::to_string(waypoint_num) + " (360 degree rotation) - ";
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            scan_msg += std::to_string(detected_cubes_.size()) + " targets found so far";
        }
        updateFeedbackStatus(scan_msg);

        // Execute rotation while processing AprilTag detections
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10); // 10 Hz control loop for smooth rotation

        while (ros::ok() && !action_server_.isPreemptRequested())
        {
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - start_time).toSec();

            // Check if rotation is complete
            if (elapsed_time >= rotation_time)
            {
                break;
            }

            // Send rotation command
            cmd_vel_pub.publish(twist_msg);

            // Process AprilTag detections during rotation
            // This is crucial for detecting tags from all angles
            ros::spinOnce();
            rate.sleep();
        }

        // Stop robot rotation
        geometry_msgs::Twist stop_msg;
        // All velocities default to 0
        cmd_vel_pub.publish(stop_msg);

        // Update feedback with scan completion status
        std::string complete_msg = "Waypoint " + std::to_string(waypoint_num) + " scan complete - ";
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            complete_msg += std::to_string(detected_cubes_.size()) + " targets found total";
        }
        updateFeedbackStatus(complete_msg);
    }

    /**
     * Update Feedback Status
     * Thread-safe method to update and publish feedback to Node A
     * Provides real-time status updates and list of found AprilTags
     * 
     * @param status Current status message describing robot's activity
     */
    void updateFeedbackStatus(const std::string &status)
    {
        std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
        feedback_.status = status;

        // Update list of found AprilTag IDs for Node A
        feedback_.found_tag_ids.clear();
        for (const auto &cube : detected_cubes_)
        {
            feedback_.found_tag_ids.push_back(cube.first);
        }

        // Send feedback to Node A
        action_server_.publishFeedback(feedback_);
    }

    /**
     * AprilTag Detection Callback
     * Processes incoming AprilTag detections continuously during navigation
     * Transforms coordinates from camera frame to map frame and stores results
     * 
     * @param msg AprilTag detection array from the camera
     */
    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        // Only process detections when navigation task is active
        // This prevents false detections when robot is idle
        if (!navigation_active_)
        {
            return;
        }

        bool new_detection = false;

        // Process each detected AprilTag in the camera frame
        for (const auto &detection : msg->detections)
        {
            int tag_id = detection.id[0];

            // Check if this tag is one of our targets (from Node A)
            if (std::find(target_ids_.begin(), target_ids_.end(), tag_id) != target_ids_.end())
            {
                try
                {
                    // Prepare coordinate transformation from camera to map frame
                    geometry_msgs::PoseStamped camera_to_tag, map_to_tag;
                    camera_to_tag.header = detection.pose.header;
                    camera_to_tag.pose = detection.pose.pose.pose;

                    // Transform detection from camera frame to global map frame
                    // This gives us the 3D position of the cube in the map coordinate system
                    tf_buffer_.transform(camera_to_tag, map_to_tag, "map", ros::Duration(1.0));

                    // Thread-safe update of detected cubes map
                    {
                        std::lock_guard<std::mutex> lock(detected_cubes_mutex_);

                        // Check if this is a genuinely new detection or position update
                        bool is_new_detection = true;
                        if (detected_cubes_.find(tag_id) != detected_cubes_.end())
                        {
                            // Compare with existing detection to avoid duplicate close detections
                            auto &existing_pose = detected_cubes_[tag_id].pose.position;
                            auto &new_pose = map_to_tag.pose.position;

                            // Calculate 3D distance between existing and new detection
                            double distance = sqrt(pow(existing_pose.x - new_pose.x, 2) +
                                                   pow(existing_pose.y - new_pose.y, 2) +
                                                   pow(existing_pose.z - new_pose.z, 2));

                            // Only update if position changed significantly (threshold: 10cm)
                            // This filters out minor detection noise
                            if (distance < 0.1)
                            {
                                is_new_detection = false;
                            }
                        }

                        // Store new or significantly updated detection
                        if (is_new_detection)
                        {
                            detected_cubes_[tag_id] = map_to_tag;
                            new_detection = true;

                            ROS_INFO("[Node B] %s AprilTag %d at map position [%.2f, %.2f, %.2f]",
                                     (detected_cubes_.size() == 1) ? "Detected target" : "Updated target",
                                     tag_id, map_to_tag.pose.position.x,
                                     map_to_tag.pose.position.y, map_to_tag.pose.position.z);
                        }
                    }
                }
                catch (tf2::TransformException &ex)
                {
                    // Log transformation failures (usually due to missing transforms)
                    ROS_WARN("[Node B] Transform failed for tag %d: %s", tag_id, ex.what());
                }
            }
        }

        // Send updated feedback if we detected new tags
        if (new_detection)
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            feedback_.found_tag_ids.clear();
            for (const auto &cube : detected_cubes_)
            {
                feedback_.found_tag_ids.push_back(cube.first);
            }
            // Update found IDs without changing the main status message
            action_server_.publishFeedback(feedback_);
        }
    }

    /**
     * Main Navigation Execution Function
     * Called when Node A sends a navigation goal
     * Executes the complete waypoint navigation and detection sequence
     * 
     * @param goal Navigation goal containing target AprilTag IDs from Node A
     */
    void executeNavigation(const assignment1_package::NavigationTaskGoalConstPtr &goal)
    {
        // Store target IDs from Node A
        target_ids_ = goal->target_ids;

        // Initialize detection storage (thread-safe)
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            detected_cubes_.clear();
        }

        // Enable continuous AprilTag detection during navigation
        navigation_active_ = true;

        updateFeedbackStatus("Starting navigation task - searching for " + std::to_string(target_ids_.size()) + " targets");

        ROS_INFO("[Node B] Starting navigation for %zu target IDs", target_ids_.size());

        // Execute waypoint navigation sequence
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            // Check for preemption or ROS shutdown
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                navigation_active_ = false;
                action_server_.setPreempted();
                return;
            }

            int waypoint_num = i + 1; // Convert to 1-indexed for user-friendly display

            // Update feedback with movement progress and current detection count
            std::string move_msg = "Moving to waypoint " + std::to_string(waypoint_num) + "/" + 
                                  std::to_string(waypoints_.size()) + " (" + 
                                  std::to_string((waypoint_num * 100) / waypoints_.size()) + "%) - ";
            {
                std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
                move_msg += std::to_string(detected_cubes_.size()) + " cubes found";
            }
            updateFeedbackStatus(move_msg);

            // Navigate to the current waypoint
            // AprilTag detection happens continuously during movement
            if (navigateToWaypoint(waypoints_[i], waypoint_num))
            {
                // Determine scanning strategy based on waypoint configuration
                if (no_rotation_waypoints_.find(waypoint_num) == no_rotation_waypoints_.end())
                {
                    // Waypoint requires comprehensive scanning - perform 360-degree rotation
                    rotateInPlace(waypoints_[i], waypoint_num);
                }
                else
                {
                    // Waypoint positioned for detection without rotation - brief pause only
                    std::string pause_msg = "Scanning waypoint " + std::to_string(waypoint_num) + " (no rotation) - ";
                    {
                        std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
                        pause_msg += std::to_string(detected_cubes_.size()) + " targets found so far";
                    }
                    updateFeedbackStatus(pause_msg);
                    
                    // Brief pause to allow detection processing without movement
                    ros::Rate rate(10);
                    ros::Time start_time = ros::Time::now();
                    while (ros::ok() && !action_server_.isPreemptRequested() && 
                           (ros::Time::now() - start_time).toSec() < 1.0) // 1-second pause
                    {
                        ros::spinOnce(); // Process AprilTag callbacks
                        rate.sleep();
                    }
                    
                    ROS_INFO("[Node B] Completed scanning at waypoint %d (no rotation)", waypoint_num);
                }
            }
        }

        // Disable continuous detection after navigation completion
        navigation_active_ = false;

        // Prepare final results for Node A
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);

            // Clear previous result data
            result_.cube_ids.clear();
            result_.cube_positions.poses.clear();

            // Package detected cubes: IDs and corresponding 3D positions
            for (const auto &cube : detected_cubes_)
            {
                result_.cube_ids.push_back(cube.first);                   // AprilTag ID
                result_.cube_positions.poses.push_back(cube.second.pose); // 3D pose in map frame
            }

            // Set coordinate frame information for pose array
            result_.cube_positions.header.frame_id = "map";
            result_.cube_positions.header.stamp = ros::Time::now();

            // Generate comprehensive final status message
            std::string final_msg = "Navigation complete! Found " + 
                                   std::to_string(detected_cubes_.size()) + "/" + 
                                   std::to_string(target_ids_.size()) + " target cubes";
            updateFeedbackStatus(final_msg);
        }

        // Send successful completion to Node A
        action_server_.setSucceeded(result_);
        ROS_INFO("[Node B] Navigation task completed! Found %zu cubes with IDs.", result_.cube_ids.size());

        // Log final results for debugging and verification
        for (size_t i = 0; i < result_.cube_ids.size(); ++i)
        {
            const auto &pos = result_.cube_positions.poses[i].position;
            ROS_INFO("[Node B] Cube ID %d at position [%.2f, %.2f, %.2f]",
                     result_.cube_ids[i], pos.x, pos.y, pos.z);
        }
    }

    /**
     * Navigate to Single Waypoint
     * Handles navigation to a specific waypoint while processing AprilTag detections
     * 
     * @param wp Waypoint coordinates (x, y) in map frame
     * @param waypoint_num Waypoint number for logging
     * @return true if navigation succeeded, false if failed or preempted
     */
    bool navigateToWaypoint(const std::pair<double, double> &wp, int waypoint_num)
    {
        // Prepare move_base goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = wp.first;
        goal.target_pose.pose.position.y = wp.second;
        goal.target_pose.pose.orientation.w = 1.0; // No specific orientation requirement

        // Send navigation goal to move_base
        move_base_client_.sendGoal(goal);

        // Active waiting loop that processes AprilTag detections during navigation
        ros::Rate rate(10); // 10 Hz processing rate
        while (!move_base_client_.waitForResult(ros::Duration(0.1)))
        {
            // Check for action preemption or ROS shutdown
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                move_base_client_.cancelGoal();
                return false;
            }

            // Process callbacks including AprilTag detections while moving
            // This enables continuous detection during transit between waypoints
            ros::spinOnce();
            rate.sleep();

            // Timeout protection - cancel if taking too long
            if ((ros::Time::now() - goal.target_pose.header.stamp).toSec() > 60.0)
            {
                ROS_WARN("[Node B] Timeout waiting for waypoint %d", waypoint_num);
                move_base_client_.cancelGoal();
                return false;
            }
        }

        // Check final navigation result
        if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("[Node B] Reached waypoint %d", waypoint_num);
            return true;
        }
        else
        {
            ROS_WARN("[Node B] Failed to reach waypoint %d", waypoint_num);
            return false;
        }
    }
};

/**
 * Main Function
 * Entry point for Node B - creates and runs the navigation action server
 */
int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "navigation_action_server");
    
    // Create navigation action server instance
    NavigationActionServer server;
    
    // Process ROS callbacks and maintain server operation
    ros::spin();
    
    return 0;
}