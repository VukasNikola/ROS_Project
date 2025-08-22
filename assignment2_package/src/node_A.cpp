#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Empty.h>
#include "tiago_iaslab_simulation/Coeffs.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "assignment2_package/GetObjectPose.h"
#include "assignment2_package/PickObject.h"
#include "assignment2_package/PlaceObject.h"

// Define some global parameters or constants
static const std::string PLANNING_GROUP_ARM = "arm";             // TIAGo arm planning group name
static const std::string PLANNING_GROUP_GRIPPER = "gripper";     // TIAGo gripper group (for opening/closing)
static const std::string BASE_FRAME = "base_footprint";          // robot base frame (or "base_link")
static const std::string TAG_FRAME = "tag_10";                   // Reference frame of AprilTag ID 10 (placing table corner)
static const std::string PLANNING_GROUP_ARM_TORSO = "arm_torso"; // For reaching deep into a scene if needed

bool navigateWithPrecision(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac,
                           const move_base_msgs::MoveBaseGoal &goal,
                           double position_tolerance = 0.1,
                           double orientation_tolerance = 0.1)
{
    ROS_INFO("Navigating to target: (%.3f, %.3f) with tolerance: pos=%.2fm, orient=%.2frad",
             goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
             position_tolerance, orientation_tolerance);

    ac.sendGoal(goal);

    // Wait for result with timeout
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout)
    {
        ROS_WARN("Navigation timed out after 30 seconds");
        ac.cancelGoal();
        return false;
    }

    actionlib::SimpleClientGoalState state = ac.getState();

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigation succeeded");

        // Brief settling time
        ros::Duration(0.5).sleep();

        // Optional: Verify we're actually at the goal
        try
        {
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Duration(0.5).sleep();

            geometry_msgs::TransformStamped robotTransform =
                tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0), ros::Duration(1.0));

            double dx = robotTransform.transform.translation.x - goal.target_pose.pose.position.x;
            double dy = robotTransform.transform.translation.y - goal.target_pose.pose.position.y;
            double dist_error = std::sqrt(dx * dx + dy * dy);

            if (dist_error > position_tolerance * 2.0)
            { // Only warn if significantly off
                ROS_WARN("Position verification: %.3fm from goal (tolerance: %.3fm)",
                         dist_error, position_tolerance);
            }
        }
        catch (...)
        {
            // Verification failed, but navigation reported success, so continue
        }

        return true;
    }
    else
    {
        ROS_ERROR("Navigation failed with state: %s", state.toString().c_str());
        return false;
    }
}

// ==================== SIMPLIFIED PLACEMENT POSE MANAGER CLASS ====================
class SimplePlacementPoseManager
{
private:
    // Reference point in tag_10 frame
    static constexpr double REF_X = 0.36;
    static constexpr double REF_Y = 0.145;

    // Best line coefficients
    double best_m_;
    double best_q_;
    bool line_found_;

    // The 3 placement points in tag_10 frame
    std::vector<std::pair<double, double>> placement_points_;

public:
    SimplePlacementPoseManager() : line_found_(false) {}

    /**
     * @brief Find the best line from 300 iterations and generate 3 placement poses
     */
    bool findBestLineAndGeneratePoses(ros::ServiceClient &coeffs_client)
    {
        ROS_INFO("Finding best line from 300 iterations...");

        double best_distance = std::numeric_limits<double>::max();
        best_m_ = 0.0;
        best_q_ = 0.0;

        for (int iteration = 0; iteration < 300; ++iteration)
        {
            tiago_iaslab_simulation::Coeffs coeffs_srv;
            coeffs_srv.request.ready = true;

            if (!coeffs_client.call(coeffs_srv))
            {
                ROS_ERROR("Failed to call /straight_line_srv on iteration %d", iteration);
                continue;
            }

            double m = coeffs_srv.response.coeffs[0];
            double q = coeffs_srv.response.coeffs[1];

            // Calculate distance from reference point (0.36, 0.145) to line y = mx + q
            // Distance formula: |mx₀ - y₀ + q| / √(m² + 1)
            double numerator = std::abs(m * REF_X - REF_Y + q);
            double denominator = std::sqrt(m * m + 1.0);
            double distance = numerator / denominator;

            ROS_INFO("Iteration %d: y = %.3fx + %.3f, distance = %.4f", iteration, m, q, distance);

            if (distance < best_distance)
            {
                best_distance = distance;
                best_m_ = m;
                best_q_ = q;
                ROS_INFO("  --> NEW BEST! Distance: %.4f", distance);
            }
        }

        ROS_INFO("===== BEST LINE FOUND =====");
        ROS_INFO("Line: y = %.3fx + %.3f", best_m_, best_q_);
        ROS_INFO("Distance from reference point: %.4f", best_distance);

        // Generate the 3 placement poses
        placement_points_.clear();

        // Pose 1: x = 0.36 - 0.15 = 0.21, y = m*0.21 + q
        double x1 = 0.36 - 0.15;
        double y1 = best_m_ * x1 + best_q_;
        placement_points_.push_back({x1, y1});

        // Pose 2: x = 0.36, y = m*0.36 + q
        double x2 = 0.36;
        double y2 = best_m_ * x2 + best_q_;
        placement_points_.push_back({x2, y2});

        // Pose 3: x = 0.36 + 0.15 = 0.51, y = m*0.51 + q
        double x3 = 0.36 + 0.15;
        double y3 = best_m_ * x3 + best_q_;
        placement_points_.push_back({x3, y3});

        ROS_INFO("Generated 3 placement poses:");
        ROS_INFO("  Pose 0: tag_10(%.3f, %.3f, 0.0)", x1, y1);
        ROS_INFO("  Pose 1: tag_10(%.3f, %.3f, 0.0)", x2, y2);
        ROS_INFO("  Pose 2: tag_10(%.3f, %.3f, 0.0)", x3, y3);

        line_found_ = true;
        return true;
    }

    /**
     * @brief Get a placement pose transformed to base_footprint frame
     * @param object_index Index of the object (0, 1, 2)
     * @param pose_out Output pose in base_footprint frame (at table surface level)
     * @param tfBuffer TF buffer for transformations
     */
    bool getPlacementPose(int object_index, geometry_msgs::PoseStamped &pose_out,
                          tf2_ros::Buffer &tfBuffer)
    {
        if (!line_found_)
        {
            ROS_ERROR("Best line not found. Call findBestLineAndGeneratePoses() first.");
            return false;
        }

        if (object_index < 0 || object_index >= 3)
        {
            ROS_ERROR("Invalid object index %d. Valid range: 0-2", object_index);
            return false;
        }

        try
        {
            ros::Time current_time = ros::Time::now();

            // Get the placement point in tag_10 frame
            double x = placement_points_[object_index].first;
            double y = placement_points_[object_index].second;

            geometry_msgs::PoseStamped pose_tag;
            pose_tag.header.frame_id = TAG_FRAME;
            pose_tag.header.stamp = current_time;
            pose_tag.pose.position.x = x;
            pose_tag.pose.position.y = y;
            pose_tag.pose.position.z = 0.0; // On the table surface in tag_10 frame

            // Simple downward-pointing orientation (let MoveIt handle the rest)
            tf2::Quaternion q_down;
            q_down.setRPY(M_PI, 0, M_PI / 2); // 180° around X (down) + +90° around Z (X forward)
            pose_tag.pose.orientation = tf2::toMsg(q_down);

            // Add gripper offset (before transformation)
            tf2::Vector3 gripper_offset(-0.05, 0.0, 0.0);
            tf2::Vector3 offset_world = tf2::quatRotate(q_down, gripper_offset);

            pose_tag.pose.position.x += offset_world.x();
            pose_tag.pose.position.y += offset_world.y();
            pose_tag.pose.position.z += offset_world.z();

            // Wait for transform to be available
            if (!tfBuffer.canTransform(BASE_FRAME, TAG_FRAME, current_time, ros::Duration(3.0)))
            {
                ROS_ERROR("Cannot get transform from %s to %s", TAG_FRAME.c_str(), BASE_FRAME.c_str());
                return false;
            }

            // Transform to base_footprint (this gives us the actual table surface position)
            tfBuffer.transform(pose_tag, pose_out, BASE_FRAME, ros::Duration(3.0));

            ROS_INFO("Placement pose %d: tag_10(%.3f, %.3f, 0.0) -> base(%.3f, %.3f, %.3f)",
                     object_index, x, y,
                     pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
            ROS_INFO("  Pose at table surface level - Node C will handle approach/place sequence");

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("TF transform error for object %d: %s", object_index, ex.what());
            return false;
        }
    }

    /**
     * @brief Check if best line has been found
     */
    bool isInitialized() const
    {
        return line_found_;
    }

    /**
     * @brief Get stored line info
     */
    void getLineInfo(double &m, double &q) const
    {
        m = best_m_;
        q = best_q_;
    }
};

// ==================== MAIN FUNCTION ====================
int main(int argc, char **argv)
{

    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;
    // ROS_INFO("Node A Sleeping for 40 seconds to ensure proper start...");
    // ros::Duration(40.0).sleep(); // Wait for 40 seconds

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Service client to get line coefficients from /straight_line_srv
    ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");

    // ==================== INITIALIZE SIMPLIFIED PLACEMENT POSE MANAGER ====================
    SimplePlacementPoseManager pose_manager; // Use the simplified version
    int num_objects = 3;

    bool best_line_found = false;      // Track if we've found the best line yet
    double best_m = 0.0, best_q = 0.0; // Store the best line coefficients

    // Initialize MoveIt interfaces for arm (for some movements), gripper, and arm_torso (for placement reach)
    moveit::planning_interface::MoveGroupInterface arm_move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_move_group(PLANNING_GROUP_GRIPPER);
    moveit::planning_interface::MoveGroupInterface arm_torso_move_group(PLANNING_GROUP_ARM_TORSO);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Initialize head action client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_client("/head_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for head controller action server...");
    head_client.waitForServer();
    ROS_INFO("Connected to head controller action server.");

    // Set tolerances for arm_torso group
    arm_torso_move_group.setGoalPositionTolerance(0.03);
    arm_torso_move_group.setGoalOrientationTolerance(0.14);
    arm_torso_move_group.setMaxAccelerationScalingFactor(0.5);
    arm_torso_move_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_move_group.setPlanningTime(10.0);

    // Also set for arm group
    arm_move_group.setGoalPositionTolerance(0.03);
    arm_move_group.setGoalOrientationTolerance(0.014);
    arm_move_group.setMaxAccelerationScalingFactor(0.5);
    arm_move_group.setMaxVelocityScalingFactor(0.5);

    // === Define an explicit "open" joint configuration ===
    std::map<std::string, double> open_position;
    open_position["gripper_left_finger_joint"] = 0.04;
    open_position["gripper_right_finger_joint"] = 0.04;

    // Open gripper
    gripper_move_group.setJointValueTarget(open_position);
    if (gripper_move_group.move())
        ROS_INFO("Node A: Gripper opened.");
    else
        ROS_WARN("Node A: Gripper open failed.");

    // Initialize navigation client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base action server.");

    // Create navigation goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "odom";

    // Service clients for Node B and Node C
    ros::ServiceClient get_pose_client = nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
    ros::ServiceClient pick_client = nh.serviceClient<assignment2_package::PickObject>("pick_object");
    ros::ServiceClient place_client = nh.serviceClient<assignment2_package::PlaceObject>("place_object");

    ROS_INFO("Waiting for services...");
    get_pose_client.waitForExistence();
    pick_client.waitForExistence();
    place_client.waitForExistence();
    ROS_INFO("All services available.");

    // TF listener to get transforms (for converting poses from tag frame to base frame)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(2.0).sleep(); // wait for TF frames to become available

    // Initial navigation to avoid table collision
    ROS_INFO("Node A: Moving to initial safe position to avoid table collision...");
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 9;
    goal.target_pose.pose.position.y = -1.0; // Safe Y position
    goal.target_pose.pose.position.z = 0.0;
    {
        tf2::Quaternion q;
        double yaw = -M_PI / 2; // Facing negative Y direction
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);
    }

    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach initial safe position. Aborting.");
        return 1;
    }
    ROS_INFO("Successfully reached initial safe position.");

    // Main pick and place loop
    int objects_processed = 0;
    while (objects_processed < num_objects && ros::ok())
    {
        ROS_INFO("Node A: Starting pick-and-place for object %d", objects_processed + 1);

        // STEP 1: Navigate to picking table
        ROS_INFO("Node A: Navigating to picking table...");
        move_base_msgs::MoveBaseGoal approach_goal;
        approach_goal.target_pose.header.frame_id = "odom";
        approach_goal.target_pose.header.stamp = ros::Time::now();
        approach_goal.target_pose.pose.position.x = 8.881;
        approach_goal.target_pose.pose.position.y = -3.02;
        approach_goal.target_pose.pose.position.z = 0.0;
        // Don't set orientation - let robot choose the easiest path
        approach_goal.target_pose.pose.orientation.w = 1.0;

        if (!navigateWithPrecision(ac, approach_goal, 0.1, 3.14))
        { // Very loose orientation tolerance
            ROS_ERROR("Failed to approach picking table");
            break;
        }

        // Step 2: Rotate in place to desired orientation
        ros::Duration(0.5).sleep(); // Let robot settle

        move_base_msgs::MoveBaseGoal rotate_goal;
        rotate_goal.target_pose.header.frame_id = "odom";
        rotate_goal.target_pose.header.stamp = ros::Time::now();
        // Keep same position
        rotate_goal.target_pose.pose.position = approach_goal.target_pose.pose.position;
        // Set desired orientation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI); // Face negative X
        rotate_goal.target_pose.pose.orientation = tf2::toMsg(q);

        if (!navigateWithPrecision(ac, rotate_goal, 0.05, 0.087))
        { // Tight tolerances for final alignment
            ROS_ERROR("Failed to align with picking table");
            break;
        }
        ROS_INFO("Successfully reached picking table.");

        // STEP 1.5: Set head position for optimal table viewing
        ROS_INFO("Node A: Setting head position for optimal table viewing...");
        try
        {
            control_msgs::FollowJointTrajectoryGoal head_goal;
            head_goal.trajectory.joint_names.push_back("head_1_joint");
            head_goal.trajectory.joint_names.push_back("head_2_joint");

            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.push_back(-0.38); // head_1_joint - rounded from -0.3819741023410197
            point.positions.push_back(-0.55); // head_2_joint - rounded from -0.5500590084104164
            point.velocities.push_back(0.0);
            point.velocities.push_back(0.0);
            point.time_from_start = ros::Duration(2.0); // 2 seconds to reach position

            head_goal.trajectory.points.push_back(point);

            head_client.sendGoal(head_goal);
            head_client.waitForResult(ros::Duration(3.0));

            if (head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Node A: Head positioned for table viewing.");
                ros::Duration(1.0).sleep(); // Allow head to settle
            }
            else
            {
                ROS_WARN("Node A: Failed to set head position, continuing anyway...");
            }
        }
        catch (...)
        {
            ROS_WARN("Node A: Exception setting head position, continuing...");
        }

        ROS_INFO("Node A: Getting object pose from Node B...");
        assignment2_package::GetObjectPose get_pose_srv;
        if (!get_pose_client.call(get_pose_srv) || get_pose_srv.response.obj_id == -1)
        {
            // Fallback if head movement fails or no objects are found
            ROS_WARN("Node A: No objects found or head movement failed. Applying fallback head pose.");

            control_msgs::FollowJointTrajectoryGoal fallback_head_goal;
            fallback_head_goal.trajectory.joint_names.push_back("head_1_joint");
            fallback_head_goal.trajectory.joint_names.push_back("head_2_joint");

            trajectory_msgs::JointTrajectoryPoint fallback_point;
            fallback_point.positions.push_back(-0.0052); // ~ Straight ahead
            fallback_point.positions.push_back(-0.7283); // ~ Tilted downward
            fallback_point.velocities.push_back(0.0);
            fallback_point.velocities.push_back(0.0);
            fallback_point.time_from_start = ros::Duration(2.0);

            fallback_head_goal.trajectory.points.push_back(fallback_point);

            head_client.sendGoal(fallback_head_goal);
            head_client.waitForResult(ros::Duration(3.0));

            ROS_INFO("Node A: Head moved to fallback position.");
            ros::Duration(1.0).sleep(); // Wait for head to settle

            // Now, try to get the object pose again
            if (!get_pose_client.call(get_pose_srv) || get_pose_srv.response.obj_id == -1)
            {
                ROS_ERROR("Node A: Failed to call get_object_pose service after fallback. Aborting.");
                break;
            }
        }

        ROS_INFO("Node A: Received object pose from Node B - ID: %d", get_pose_srv.response.obj_id);

        // STEP 3: Call Node C to pick the object
        ROS_INFO("Node A: Calling Node C to pick object...");
        assignment2_package::PickObject pick_srv;
        pick_srv.request.target = get_pose_srv.response.obj_pose;
        pick_srv.request.target_id = get_pose_srv.response.obj_id;

        if (!pick_client.call(pick_srv) || !pick_srv.response.success)
        {
            ROS_ERROR("Node A: Failed to pick object %d. Moving to safe position and continuing to next object.", objects_processed + 1);

            // Move back to safe position before trying next object
            ROS_INFO("Node A: Returning to safe position after failed pick...");
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 9;
            goal.target_pose.pose.position.y = -1.0;
            goal.target_pose.pose.position.z = 0.0;
            {
                tf2::Quaternion q;
                double yaw = -M_PI / 2; // Facing negative Y direction
                q.setRPY(0.0, 0.0, yaw);
                goal.target_pose.pose.orientation = tf2::toMsg(q);
            }

            ac.sendGoal(goal);
            ac.waitForResult();

            // Add delay to allow objects to be re-detected
            ros::Duration(3.0).sleep();

            continue; // Try next object
        }
        ROS_INFO("Node A: Object %d picked successfully!", objects_processed + 1);

        ROS_INFO("Node A: Waiting for planning scene to stabilize after pick...");
        ros::Duration(2.0).sleep(); // 3 second delay to let Node C finish its updates


        // NEW STEP 3.7: Rotate to face positive Y direction to avoid arc navigation
        ROS_INFO("Node A: Rotating to face positive Y direction for straight navigation...");
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 8.881; // Keep same position
        goal.target_pose.pose.position.y = -3.02; // Keep same position
        goal.target_pose.pose.position.z = 0.0;
        {
            tf2::Quaternion q;
            double yaw = M_PI / 2; // Face positive Y direction
            q.setRPY(0.0, 0.0, yaw);
            goal.target_pose.pose.orientation = tf2::toMsg(q);
        }

        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("Failed to rotate to positive Y, continuing anyway.");
        }
        else
        {
            ROS_INFO("Successfully rotated to face positive Y direction.");
        }

        // STEP 4: Navigate to placing table (single move with tight tolerances)
        ROS_INFO("Node A: Navigating to placing table...");
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 9;
        goal.target_pose.pose.position.y = -1.92; // Correct Y for placing table
        goal.target_pose.pose.position.z = 0.0;

        // Set precise orientation (facing negative X direction)
        tf2::Quaternion q_place;
        q_place.setRPY(0.0, 0.0, M_PI); // Facing negative X direction
        goal.target_pose.pose.orientation = tf2::toMsg(q_place);

        if (!navigateWithPrecision(ac, goal, 0.03, 0.05))
        {
            ROS_ERROR("Failed to reach placing table with precise navigation. Aborting.");
            break;
        }
        ROS_INFO("Successfully reached placing table with precise navigation.");

        // Allow time for robot to settle and any dynamic elements to stabilize
        ROS_INFO("Node A: Waiting for robot to settle at placing table...");
        ros::Duration(2.0).sleep(); // 2 second delay

        // STEP 4.5: Set head position to see tag_10
        ROS_INFO("Node A: Setting head position to see tag_10...");
        try
        {
            control_msgs::FollowJointTrajectoryGoal head_goal;
            head_goal.trajectory.joint_names.push_back("head_1_joint");
            head_goal.trajectory.joint_names.push_back("head_2_joint");

            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.push_back(0.176);  // head_1_joint - rounded from 0.17575005816878292
            point.positions.push_back(-0.710); // head_2_joint - rounded from -0.7100389700865404
            point.velocities.push_back(0.0);
            point.velocities.push_back(0.0);
            point.time_from_start = ros::Duration(2.0); // 2 seconds to reach position

            head_goal.trajectory.points.push_back(point);

            head_client.sendGoal(head_goal);
            head_client.waitForResult(ros::Duration(3.0));

            if (head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Node A: Head positioned to see tag_10.");
                ros::Duration(1.0).sleep(); // Allow head to settle and tag detection
            }
            else
            {
                ROS_WARN("Node A: Failed to set head position for tag_10, continuing anyway...");
            }
        }
        catch (...)
        {
            ROS_WARN("Node A: Exception setting head position for tag_10, continuing...");
        }

        // STEP 4.6: Find best line on FIRST visit to placing table
        if (!pose_manager.isInitialized()) // Much simpler check
        {
            ROS_INFO("Node A: First time at placing table - finding best line from 300 iterations...");

            // Wait a bit to ensure tag_10 is being detected
            ros::Duration(2.0).sleep();

            // This single call does everything: finds best line AND generates 3 poses
            if (!pose_manager.findBestLineAndGeneratePoses(coeffs_client))
            {
                ROS_ERROR("Node A: Failed to find best line and generate poses");
                break;
            }

            ROS_INFO("Node A: Best line found and 3 placement poses generated!");
        }
        else
        {
            double m, q;
            pose_manager.getLineInfo(m, q);
            ROS_INFO("Node A: Using previously found best line y = %.3fx + %.3f", m, q);
        }

        // STEP 5: Get placement pose (at table surface level)
        ROS_INFO("Node A: Getting placement pose for object %d...", objects_processed + 1);

        geometry_msgs::PoseStamped placement_pose_base;
        if (!pose_manager.getPlacementPose(objects_processed, placement_pose_base, tfBuffer))
        {
            ROS_ERROR("Node A: Failed to get placement pose for object %d. Skipping.", objects_processed + 1);
            continue;
        }

        // STEP 6: Call Node C to place the object (it will handle the final positioning)
        ROS_INFO("Node A: Calling Node C to place object from approach pose...");
        assignment2_package::PlaceObject place_srv;
        place_srv.request.target_id = get_pose_srv.response.obj_id;
        place_srv.request.target_pose = placement_pose_base;

        if (!place_client.call(place_srv) || !place_srv.response.success)
        {
            ROS_ERROR("Node A: Failed to place object %d.", objects_processed + 1);
            // Continue anyway - object might still be placed
        }
        else
        {
            ROS_INFO("Node A: Object %d placed successfully!", objects_processed + 1);
        }

        if (objects_processed < num_objects - 1) // Only if there are more objects to pick
        {
            ROS_INFO("Node A: Waiting for planning scene to stabilize after place...");
            ros::Duration(2.0).sleep(); // 2 second delay
            // NEW STEP 6.7: Rotate to face negative Y direction to avoid arc navigation
            ROS_INFO("Node A: Rotating to face negative Y direction for straight navigation...");
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 9;     // Keep same position
            goal.target_pose.pose.position.y = -1.92; // Keep same position
            goal.target_pose.pose.position.z = 0.0;
            {
                tf2::Quaternion q;
                double yaw = -M_PI / 2; // Face negative Y direction
                q.setRPY(0.0, 0.0, yaw);
                goal.target_pose.pose.orientation = tf2::toMsg(q);
            }

            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_WARN("Failed to rotate to negative Y, continuing anyway.");
            }
            else
            {
                ROS_INFO("Successfully rotated to face negative Y direction.");
            }
        }

        objects_processed++;
        ROS_INFO("Node A: Completed pick-and-place for object %d/%d", objects_processed, num_objects);
    }

    ROS_INFO("Node A: Pick-and-place routine completed. Processed %d objects.", objects_processed);
    ros::shutdown();
    return 0;
}