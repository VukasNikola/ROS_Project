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

    assignment1_package::NavigationTaskFeedback feedback_;
    assignment1_package::NavigationTaskResult result_;

public:
    NavigationActionServer() : action_server_(nh_, "navigation_task", boost::bind(&NavigationActionServer::executeNavigation, this, _1), false),
                               move_base_client_("move_base", true),
                               head_client_("/head_controller/follow_joint_trajectory", true),
                               tf_listener_(tf_buffer_)
    {

        // Load waypoints
        loadWaypoints();

        // Subscribe to AprilTag detections
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
            {-0.09, -1.01}, {3.08, 0.00}, {6.58, 0.63}, {7.18, -0.77}, {8.58, -2.37}, {7.58, -3.37}, {8.88, -3.97}, {10.58, -4.07}, {11.58, -2.87}, {13.08, -2.87}, {13.08, -1.37}, {12.08, 1.13}, {11.08, 1.13}, {10.58, 0.13}, {8.78, 0.63}};
        ROS_INFO("[Node B] Loaded %zu hardcoded waypoints", waypoints_.size());
    }

    void moveHeadDown()
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.0, -0.75}; // Pan=0, Tilt down
        point.time_from_start = ros::Duration(2.0);
        goal.trajectory.points.push_back(point);

        head_client_.sendGoal(goal);
        head_client_.waitForResult();
        ROS_INFO("[Node B] Head moved down for cube detection");
    }

    void rotateInPlace(const std::pair<double, double> &position, int waypoint_num)
    {
        ROS_INFO("[Node B] Performing 360 degree scan at waypoint %d", waypoint_num);

        // Get current robot pose from TF
        geometry_msgs::TransformStamped robot_transform;
        try
        {
            robot_transform = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("[Node B] Could not get robot transform: %s", ex.what());
            return;
        }

        // Extract current yaw
        tf2::Quaternion current_q(
            robot_transform.transform.rotation.x,
            robot_transform.transform.rotation.y,
            robot_transform.transform.rotation.z,
            robot_transform.transform.rotation.w);
        double roll, pitch, current_yaw;
        tf2::Matrix3x3(current_q).getRPY(roll, pitch, current_yaw);

        // Two rotations:
        // First: 185-190 degrees (slightly more than 180 to ensure same direction)
        // Second: Complete the remaining ~170-175 degrees
        std::vector<double> rotation_angles = {
            current_yaw + (185.0 * M_PI / 180.0), // First rotation: 185 degrees
            current_yaw + (360.0 * M_PI / 180.0)  // Second rotation: complete the circle
        };

        for (size_t i = 0; i < rotation_angles.size(); ++i)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = position.first;
            goal.target_pose.pose.position.y = position.second;
            goal.target_pose.pose.position.z = 0.0;

            // Normalize angle to [-pi, pi]
            double target_yaw = rotation_angles[i];
            while (target_yaw > M_PI)
                target_yaw -= 2 * M_PI;
            while (target_yaw < -M_PI)
                target_yaw += 2 * M_PI;

            tf2::Quaternion q;
            q.setRPY(0, 0, target_yaw);
            goal.target_pose.pose.orientation.x = q.x();
            goal.target_pose.pose.orientation.y = q.y();
            goal.target_pose.pose.orientation.z = q.z();
            goal.target_pose.pose.orientation.w = q.w();

            feedback_.status = "Scanning rotation " + std::to_string(i + 1) + "/2 at waypoint " + std::to_string(waypoint_num);
            action_server_.publishFeedback(feedback_);

            move_base_client_.sendGoal(goal);
            bool success = move_base_client_.waitForResult(ros::Duration(20.0));

            if (success && move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("[Node B] Completed rotation %zu/2 (%.1f degrees total)",
                         i + 1, (i == 0 ? 185.0 : 360.0));

                // Pause for AprilTag detection
                ros::Duration(2.0).sleep();
                ros::spinOnce();
            }
            else
            {
                ROS_WARN("[Node B] Failed rotation %zu/2", i + 1);
            }
        }

        ROS_INFO("[Node B] 360 degree scan completed at waypoint %d", waypoint_num);
    }

    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
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

                    detected_cubes_[tag_id] = map_to_tag;

                    // Update feedback
                    feedback_.status = "Found AprilTag with ID " + std::to_string(tag_id);
                    feedback_.found_tag_ids.clear();
                    for (const auto &cube : detected_cubes_)
                    {
                        feedback_.found_tag_ids.push_back(cube.first);
                    }
                    action_server_.publishFeedback(feedback_);

                    ROS_INFO("[Node B] Detected target AprilTag %d at map position [%.2f, %.2f, %.2f]",
                             tag_id, map_to_tag.pose.position.x, map_to_tag.pose.position.y, map_to_tag.pose.position.z);
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("[Node B] Transform failed: %s", ex.what());
                }
            }
        }
    }

    void executeNavigation(const assignment1_package::NavigationTaskGoalConstPtr &goal)
    {
        target_ids_ = goal->target_ids;
        detected_cubes_.clear();

        feedback_.status = "Starting navigation task";
        feedback_.found_tag_ids.clear();
        action_server_.publishFeedback(feedback_);

        ROS_INFO("[Node B] Starting navigation for %zu target IDs", target_ids_.size());

        // Navigate through waypoints
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                action_server_.setPreempted();
                return;
            }

            // Update feedback - moving to waypoint
            feedback_.status = "Moving to waypoint " + std::to_string(i + 1) + "/" + std::to_string(waypoints_.size());
            action_server_.publishFeedback(feedback_);

            // Navigate to waypoint
            if (navigateToWaypoint(waypoints_[i], i + 1))
            {
                // Update feedback - scanning
                feedback_.status = "Scanning 360° for AprilTags at waypoint " + std::to_string(i + 1);
                action_server_.publishFeedback(feedback_);

                // Rotate 360° at this position
                rotateInPlace(waypoints_[i], i + 1);
            }
        }

        // Prepare result
        result_.cube_positions.poses.clear();
        for (const auto &cube : detected_cubes_)
        {
            result_.cube_positions.poses.push_back(cube.second.pose);
        }

        feedback_.status = "Detection finished. Found " + std::to_string(detected_cubes_.size()) + " cubes.";
        action_server_.publishFeedback(feedback_);

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
        bool success = move_base_client_.waitForResult(ros::Duration(60.0));

        if (success && move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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