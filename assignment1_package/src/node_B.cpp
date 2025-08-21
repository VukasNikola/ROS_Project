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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<assignment1_package::NavigationTaskAction> NavigationServer;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadClient;

class NavigationActionServer
{
private:
    ros::NodeHandle nh_;
    NavigationServer action_server_;
    MoveBaseClient move_base_client_;
    HeadClient head_client_;
    ros::Subscriber apriltag_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<std::pair<double, double>> waypoints_;
    std::vector<int32_t> target_ids_;
    std::map<int, geometry_msgs::PoseStamped> detected_cubes_;
    std::mutex detected_cubes_mutex_; // Thread safety for continuous detection

    assignment1_package::NavigationTaskFeedback feedback_;
    assignment1_package::NavigationTaskResult result_;

    bool navigation_active_; // Flag to control detection behavior

public:
    NavigationActionServer() : action_server_(nh_, "navigation_task", boost::bind(&NavigationActionServer::executeNavigation, this, _1), false),
                               move_base_client_("move_base", true),
                               head_client_("/head_controller/follow_joint_trajectory", true),
                               tf_listener_(tf_buffer_),
                               navigation_active_(false)
    {

        // Load waypoints
        loadWaypoints();

        // Subscribe to AprilTag detections - now runs continuously
        apriltag_sub_ = nh_.subscribe("/tag_detections", 1,
                                      &NavigationActionServer::apriltagCallback, this);

        // Wait for move_base
        ROS_INFO("[Node B] Waiting for move_base action server...");
        move_base_client_.waitForServer();

        // Wait for head controller
        ROS_INFO("[Node B] Waiting for head controller...");
        head_client_.waitForServer();

        // Move head down at startup
        moveHeadDown();

        action_server_.start();
        ROS_INFO("[Node B] Navigation action server started!");
    }

    void loadWaypoints()
    {
        waypoints_ = {
            {-0.09, -1.01}, {7.18, -0.77}, {8.58, -2.37}, {8, -3.37}, {12.15, -2.87}, {13.08, -1.37}, {12.08, 1.13}, {10.70, 0.70}, {8.78, 0.63}};
        ROS_INFO("[Node B] Loaded %zu hardcoded waypoints", waypoints_.size());
    }

    void moveHeadDown()
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.0, -1.3}; // Pan=0, Tilt down
        point.time_from_start = ros::Duration(2.0);
        goal.trajectory.points.push_back(point);

        head_client_.sendGoal(goal);
        head_client_.waitForResult();
        ROS_INFO("[Node B] Head moved down for cube detection");
    }

    void rotateInPlace(const std::pair<double, double> &position, int waypoint_num)
    {
        ROS_INFO("[Node B] Performing 360 degree scan at waypoint %d", waypoint_num);

        // Create velocity publisher for direct rotation control
        ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        ros::Duration(0.1).sleep(); // Brief wait for publisher to connect

        // Rotation parameters
        double angular_velocity = 0.5;  // rad/s (adjust as needed)
        double target_angle = 2 * M_PI; // Full circle (360 degrees)

        // Calculate rotation time needed
        double rotation_time = target_angle / angular_velocity;

        // Create Twist message for rotation
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_velocity; // Positive for counter-clockwise

        // Update feedback
        updateFeedbackStatus("Rotating 360 degrees for AprilTag detection at waypoint " + std::to_string(waypoint_num));

        // Record start time
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10); // 10 Hz

        // Publish rotation commands
        while (ros::ok() && !action_server_.isPreemptRequested())
        {
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - start_time).toSec();

            if (elapsed_time >= rotation_time)
            {
                break;
            }

            cmd_vel_pub.publish(twist_msg);

            // Process AprilTag callbacks during rotation
            ros::spinOnce();
            rate.sleep();
        }

        // Stop the robot
        geometry_msgs::Twist stop_msg;
        // All velocities are 0 by default
        cmd_vel_pub.publish(stop_msg);

        ROS_INFO("[Node B] 360 degree scan completed at waypoint %d", waypoint_num);
    }

    void updateFeedbackStatus(const std::string &status)
    {
        std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
        feedback_.status = status;

        // Update found tag IDs
        feedback_.found_tag_ids.clear();
        for (const auto &cube : detected_cubes_)
        {
            feedback_.found_tag_ids.push_back(cube.first);
        }

        action_server_.publishFeedback(feedback_);
    }

    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        // Only process detections when navigation is active
        if (!navigation_active_)
        {
            return;
        }

        bool new_detection = false;

        for (const auto &detection : msg->detections)
        {
            int tag_id = detection.id[0];

            // Check if this is one of our target IDs
            if (std::find(target_ids_.begin(), target_ids_.end(), tag_id) != target_ids_.end())
            {
                try
                {
                    geometry_msgs::PoseStamped camera_to_tag, map_to_tag;
                    camera_to_tag.header = detection.pose.header;
                    camera_to_tag.pose = detection.pose.pose.pose;

                    // Transform directly from camera frame to map frame
                    tf_buffer_.transform(camera_to_tag, map_to_tag, "map", ros::Duration(1.0));

                    // Thread-safe update of detected cubes
                    {
                        std::lock_guard<std::mutex> lock(detected_cubes_mutex_);

                        // Check if this is a new detection or significantly different position
                        bool is_new_detection = true;
                        if (detected_cubes_.find(tag_id) != detected_cubes_.end())
                        {
                            auto &existing_pose = detected_cubes_[tag_id].pose.position;
                            auto &new_pose = map_to_tag.pose.position;

                            // Calculate distance between existing and new detection
                            double distance = sqrt(pow(existing_pose.x - new_pose.x, 2) +
                                                   pow(existing_pose.y - new_pose.y, 2) +
                                                   pow(existing_pose.z - new_pose.z, 2));

                            // Only update if position changed significantly (> 10cm)
                            if (distance < 0.1)
                            {
                                is_new_detection = false;
                            }
                        }

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
                    ROS_WARN("[Node B] Transform failed for tag %d: %s", tag_id, ex.what());
                }
            }
        }

        // Update feedback if we had new detections
        if (new_detection)
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            feedback_.found_tag_ids.clear();
            for (const auto &cube : detected_cubes_)
            {
                feedback_.found_tag_ids.push_back(cube.first);
            }
            // Don't change the main status message, just update the found IDs
            action_server_.publishFeedback(feedback_);
        }
    }

    void executeNavigation(const assignment1_package::NavigationTaskGoalConstPtr &goal)
    {
        target_ids_ = goal->target_ids;

        // Thread-safe initialization
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            detected_cubes_.clear();
        }

        navigation_active_ = true; // Enable continuous detection

        updateFeedbackStatus("Starting navigation task");

        ROS_INFO("[Node B] Starting navigation for %zu target IDs", target_ids_.size());

        // Navigate through waypoints with continuous detection
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                navigation_active_ = false;
                action_server_.setPreempted();
                return;
            }

            // Update feedback - moving to waypoint
            updateFeedbackStatus("Moving to waypoint " + std::to_string(i + 1) + "/" + std::to_string(waypoints_.size()));

            // Navigate to waypoint (detection happens continuously during movement)
            if (navigateToWaypoint(waypoints_[i], i + 1))
            {
                // Update feedback - scanning
                updateFeedbackStatus("Scanning 360 degrees for AprilTags at waypoint " + std::to_string(i + 1));

                // Rotate 360° at this position (additional detection opportunity)
                rotateInPlace(waypoints_[i], i + 1);
            }
        }

        navigation_active_ = false; // Disable continuous detection

        // Prepare result
        {
            std::lock_guard<std::mutex> lock(detected_cubes_mutex_);
            result_.cube_positions.poses.clear();
            for (const auto &cube : detected_cubes_)
            {
                result_.cube_positions.poses.push_back(cube.second.pose);
            }
        }

        updateFeedbackStatus("Detection finished. Found " + std::to_string(detected_cubes_.size()) + " cubes.");

        action_server_.setSucceeded(result_);
        ROS_INFO("[Node B] Navigation task completed!");
    }

    bool navigateToWaypoint(const std::pair<double, double> &wp, int waypoint_num)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = wp.first;
        goal.target_pose.pose.position.y = wp.second;
        goal.target_pose.pose.orientation.w = 1.0;

        move_base_client_.sendGoal(goal);

        // Instead of just waiting, actively spin to process AprilTag callbacks
        ros::Rate rate(10); // 10 Hz
        while (!move_base_client_.waitForResult(ros::Duration(0.1)))
        {
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                move_base_client_.cancelGoal();
                return false;
            }

            // Process callbacks (including AprilTag detections) while moving
            ros::spinOnce();
            rate.sleep();

            // Check if we've been trying for too long
            if ((ros::Time::now() - goal.target_pose.header.stamp).toSec() > 60.0)
            {
                ROS_WARN("[Node B] Timeout waiting for waypoint %d", waypoint_num);
                move_base_client_.cancelGoal();
                return false;
            }
        }

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_action_server");
    NavigationActionServer server;
    ros::spin();
    return 0;
}