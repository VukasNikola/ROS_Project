/**
 * @file node_A.cpp
 * @brief Main pick and place coordination node
 * 
 * This node orchestrates the complete pick-and-place workflow:
 * - Navigation between picking and placing tables
 * - Head positioning for optimal AprilTag detection
 * - Coordination with Node B (object detection) and Node C (manipulation)
 * - Placement pose calculation using straight line detection
 * - Sequential processing of objects by type priority
 * 
 * Workflow Overview:
 * 1. Navigate to picking table with precise positioning
 * 2. Position head for object detection
 * 3. Request object pose from Node B (priority-based selection)
 * 4. Call Node C to pick the object
 * 5. Navigate to placing table with collision avoidance
 * 6. Find optimal placement line (first visit only)
 * 7. Calculate placement pose and call Node C to place object
 * 8. Repeat for all objects (typically 3)
 * 
 * Navigation Strategy:
 * - Uses separate approach and rotation moves for precise final positioning
 * - Implements intermediate orientations to avoid arc trajectories
 * - Includes settling delays for dynamic stability
 */

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

// =============================================================================
// CONSTANTS AND CONFIGURATION
// =============================================================================

// MoveIt planning group names
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string PLANNING_GROUP_ARM_TORSO = "arm_torso";

// Reference frames
static const std::string BASE_FRAME = "base_footprint";
static const std::string TAG_FRAME = "tag_10"; // AprilTag ID 10 on placing table

// Task configuration
static const int NUM_OBJECTS = 3; // Total objects to process

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * @brief Navigate to target with precision control and verification
 * 
 * Provides enhanced navigation with:
 * - Configurable position and orientation tolerances
 * - Timeout handling with goal cancellation
 * - Optional position verification after completion
 * - Settling time for dynamic stability
 * 
 * @param ac Navigation action client
 * @param goal Target pose in map frame
 * @param position_tolerance Acceptable position error (meters)
 * @param orientation_tolerance Acceptable orientation error (radians)
 * @return true if navigation succeeded, false otherwise
 */
bool navigateWithPrecision(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac,
                           const move_base_msgs::MoveBaseGoal &goal,
                           double position_tolerance = 0.1,
                           double orientation_tolerance = 0.1)
{
    ROS_INFO("Navigating to target: (%.3f, %.3f) with tolerance: pos=%.2fm, orient=%.2frad",
             goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
             position_tolerance, orientation_tolerance);

    ac.sendGoal(goal);

    // Wait for result with timeout protection
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
        // Brief settling time for dynamic stability
        ros::Duration(0.5).sleep();

        // Optional position verification for critical movements
        try
        {
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Duration(0.5).sleep();

            geometry_msgs::TransformStamped robotTransform =
                tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));

            double dx = robotTransform.transform.translation.x - goal.target_pose.pose.position.x;
            double dy = robotTransform.transform.translation.y - goal.target_pose.pose.position.y;
            double dist_error = std::sqrt(dx * dx + dy * dy);

            if (dist_error > position_tolerance * 2.0)
            {
                ROS_WARN("Position verification: %.3fm from goal (tolerance: %.3fm)",
                         dist_error, position_tolerance);
            }
        }
        catch (...)
        {
            // Verification failed, but navigation reported success, continue
        }

        return true;
    }
    else
    {
        ROS_ERROR("Navigation failed with state: %s", state.toString().c_str());
        return false;
    }
}

// =============================================================================
// PLACEMENT POSE MANAGEMENT CLASS
// =============================================================================

/**
 * @brief Manages placement pose calculation using straight line detection
 * 
 * This class handles:
 * - Finding the optimal straight line through 300 iterations
 * - Generating 3 evenly spaced placement poses along the line
 * - Transforming poses from tag_10 frame to base_footprint frame
 * - Caching results to avoid recalculation
 * 
 * Line Detection Strategy:
 * - Reference point: (0.36, 0.145) in tag_10 frame
 * - Finds line with minimum distance to reference point
 * - Generates poses at x-offsets: -0.15, 0.0, +0.15 from reference
 */
class SimplePlacementPoseManager
{
private:
    // Reference point in tag_10 frame for line optimization
    static constexpr double REF_X = 0.36;
    static constexpr double REF_Y = 0.145;

    // Cached line coefficients (y = mx + q)
    double best_m_;
    double best_q_;
    bool line_found_;

    // Generated placement points in tag_10 frame
    std::vector<std::pair<double, double>> placement_points_;

public:
    SimplePlacementPoseManager() : line_found_(false) {}

    /**
     * @brief Find optimal line through 300 iterations and generate placement poses
     * 
     * Process:
     * 1. Query straight line service 300 times
     * 2. Calculate distance from each line to reference point
     * 3. Select line with minimum distance
     * 4. Generate 3 placement poses along the best line
     * 
     * @param coeffs_client Service client for line coefficient queries
     * @return true if successful, false otherwise
     */
    bool findBestLineAndGeneratePoses(ros::ServiceClient &coeffs_client)
    {
        ROS_INFO("Finding best line from 300 iterations...");

        double best_distance = std::numeric_limits<double>::max();
        best_m_ = 0.0;
        best_q_ = 0.0;

        // Iterate through line detections to find optimal placement line
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

            // Calculate perpendicular distance from reference point to line y = mx + q
            // Distance formula: |mx₀ - y₀ + q| / √(m² + 1)
            double numerator = std::abs(m * REF_X - REF_Y + q);
            double denominator = std::sqrt(m * m + 1.0);
            double distance = numerator / denominator;

            if (distance < best_distance)
            {
                best_distance = distance;
                best_m_ = m;
                best_q_ = q;
                ROS_INFO("New best line found: y = %.3fx + %.3f, distance = %.4f", m, q, distance);
            }
        }

        ROS_INFO("===== OPTIMAL LINE FOUND =====");
        ROS_INFO("Line equation: y = %.3fx + %.3f", best_m_, best_q_);
        ROS_INFO("Distance from reference point: %.4f", best_distance);

        // Generate 3 evenly spaced placement poses along the optimal line
        placement_points_.clear();

        // Pose 1: Left offset (x = 0.36 - 0.15 = 0.21)
        double x1 = 0.36 - 0.15;
        double y1 = best_m_ * x1 + best_q_;
        placement_points_.push_back({x1, y1});

        // Pose 2: Center (x = 0.36)
        double x2 = 0.36;
        double y2 = best_m_ * x2 + best_q_;
        placement_points_.push_back({x2, y2});

        // Pose 3: Right offset (x = 0.36 + 0.15 = 0.51)
        double x3 = 0.36 + 0.15;
        double y3 = best_m_ * x3 + best_q_;
        placement_points_.push_back({x3, y3});

        ROS_INFO("Generated placement poses in tag_10 frame:");
        ROS_INFO("  Pose 0: (%.3f, %.3f)", x1, y1);
        ROS_INFO("  Pose 1: (%.3f, %.3f)", x2, y2);
        ROS_INFO("  Pose 2: (%.3f, %.3f)", x3, y3);

        line_found_ = true;
        return true;
    }

    /**
     * @brief Get placement pose transformed to base_footprint frame
     * 
     * Transforms placement pose from tag_10 frame to base_footprint frame:
     * 1. Apply gripper offset in tag_10 frame
     * 2. Set appropriate orientation for manipulation
     * 3. Transform complete pose to base_footprint frame
     * 4. Result is at table surface level (Node C adds object-specific offsets)
     * 
     * @param object_index Index of placement pose (0, 1, or 2)
     * @param pose_out Output pose in base_footprint frame
     * @param tfBuffer TF buffer for coordinate transformations
     * @return true if successful, false otherwise
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

            // Get pre-calculated placement point in tag_10 frame
            double x = placement_points_[object_index].first;
            double y = placement_points_[object_index].second;

            geometry_msgs::PoseStamped pose_tag;
            pose_tag.header.frame_id = TAG_FRAME;
            pose_tag.header.stamp = current_time;
            pose_tag.pose.position.x = x;
            pose_tag.pose.position.y = y;
            pose_tag.pose.position.z = 0.0; // Table surface level in tag_10 frame

            // Set orientation for downward-pointing gripper approach
            tf2::Quaternion q_down;
            q_down.setRPY(M_PI, 0, M_PI / 2); // 180° around X (down) + 90° around Z (align)
            pose_tag.pose.orientation = tf2::toMsg(q_down);

            // Apply gripper offset before transformation
            tf2::Vector3 gripper_offset(-0.05, 0.0, 0.0);
            tf2::Vector3 offset_world = tf2::quatRotate(q_down, gripper_offset);

            pose_tag.pose.position.x += offset_world.x();
            pose_tag.pose.position.y += offset_world.y();
            pose_tag.pose.position.z += offset_world.z();

            // Ensure transform is available
            if (!tfBuffer.canTransform(BASE_FRAME, TAG_FRAME, current_time, ros::Duration(3.0)))
            {
                ROS_ERROR("Cannot get transform from %s to %s", TAG_FRAME.c_str(), BASE_FRAME.c_str());
                return false;
            }

            // Transform to base_footprint frame
            tfBuffer.transform(pose_tag, pose_out, BASE_FRAME, ros::Duration(3.0));

            ROS_INFO("Placement pose %d: tag_10(%.3f, %.3f) -> base(%.3f, %.3f, %.3f)",
                     object_index, x, y,
                     pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("TF transform error for object %d: %s", object_index, ex.what());
            return false;
        }
    }

    /**
     * @brief Check if optimal line has been found and poses generated
     */
    bool isInitialized() const
    {
        return line_found_;
    }

    /**
     * @brief Get cached line coefficients for information
     */
    void getLineInfo(double &m, double &q) const
    {
        m = best_m_;
        q = best_q_;
    }
};

// =============================================================================
// MAIN FUNCTION
// =============================================================================

/**
 * @brief Main coordination function for pick-and-place workflow
 * 
 * Complete workflow:
 * 1. Initialize all interfaces (MoveIt, navigation, services)
 * 2. Open gripper and move to safe starting position
 * 3. For each object (typically 3):
 *    a. Navigate to picking table with precise positioning
 *    b. Position head for optimal object detection
 *    c. Get object pose from Node B (priority selection)
 *    d. Call Node C to pick object
 *    e. Navigate to placing table avoiding obstacles
 *    f. Find optimal placement line (first visit only)
 *    g. Calculate placement pose and call Node C to place
 * 4. Complete mission and shutdown
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // =============================================================================
    // INITIALIZATION PHASE
    // =============================================================================

    ROS_INFO("Node A: Initializing pick-and-place coordinator...");

    // Initialize service clients
    ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    ros::ServiceClient get_pose_client = nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
    ros::ServiceClient pick_client = nh.serviceClient<assignment2_package::PickObject>("pick_object");
    ros::ServiceClient place_client = nh.serviceClient<assignment2_package::PlaceObject>("place_object");

    // Initialize placement pose manager
    SimplePlacementPoseManager pose_manager;

    // Initialize MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface arm_move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_move_group(PLANNING_GROUP_GRIPPER);
    moveit::planning_interface::MoveGroupInterface arm_torso_move_group(PLANNING_GROUP_ARM_TORSO);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Initialize head action client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_client("/head_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for head controller...");
    head_client.waitForServer();

    // Configure MoveIt planning parameters
    arm_torso_move_group.setGoalPositionTolerance(0.03);
    arm_torso_move_group.setGoalOrientationTolerance(0.14);
    arm_torso_move_group.setMaxAccelerationScalingFactor(0.5);
    arm_torso_move_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_move_group.setPlanningTime(10.0);

    arm_move_group.setGoalPositionTolerance(0.03);
    arm_move_group.setGoalOrientationTolerance(0.014);
    arm_move_group.setMaxAccelerationScalingFactor(0.5);
    arm_move_group.setMaxVelocityScalingFactor(0.5);

    // Open gripper for initial state
    std::map<std::string, double> open_position;
    open_position["gripper_left_finger_joint"] = 0.04;
    open_position["gripper_right_finger_joint"] = 0.04;

    gripper_move_group.setJointValueTarget(open_position);
    if (gripper_move_group.move())
        ROS_INFO("Gripper opened successfully");
    else
        ROS_WARN("Failed to open gripper");

    // Initialize navigation client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for navigation server...");
    ac.waitForServer();

    // Wait for all required services
    ROS_INFO("Waiting for required services...");
    get_pose_client.waitForExistence();
    pick_client.waitForExistence();
    place_client.waitForExistence();

    // Initialize TF system
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(2.0).sleep(); // Allow TF frames to populate

    ROS_INFO("All systems initialized successfully");

    // =============================================================================
    // INITIAL NAVIGATION TO SAFE POSITION
    // =============================================================================

    ROS_INFO("Moving to initial safe position...");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 9;
    goal.target_pose.pose.position.y = -1.0;
    goal.target_pose.pose.position.z = 0.0;

    tf2::Quaternion q_safe;
    q_safe.setRPY(0.0, 0.0, -M_PI / 2); // Face negative Y direction
    goal.target_pose.pose.orientation = tf2::toMsg(q_safe);

    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach initial safe position. Aborting.");
        return 1;
    }

    // =============================================================================
    // MAIN PICK-AND-PLACE LOOP
    // =============================================================================

    int objects_processed = 0;
    while (objects_processed < NUM_OBJECTS && ros::ok())
    {
        ROS_INFO("=== Starting pick-and-place for object %d/%d ===", objects_processed + 1, NUM_OBJECTS);

        // -------------------------------------------------------------------------
        // PHASE 1: NAVIGATION TO PICKING TABLE
        // -------------------------------------------------------------------------

        ROS_INFO("Phase 1: Navigating to picking table...");

        // Step 1a: Approach picking table position
        move_base_msgs::MoveBaseGoal approach_goal;
        approach_goal.target_pose.header.frame_id = "map";
        approach_goal.target_pose.header.stamp = ros::Time::now();
        approach_goal.target_pose.pose.position.x = 8.881;
        approach_goal.target_pose.pose.position.y = -3.02;
        approach_goal.target_pose.pose.position.z = 0.0;
        approach_goal.target_pose.pose.orientation.w = 1.0; // Let robot choose orientation

        if (!navigateWithPrecision(ac, approach_goal, 0.1, 3.14))
        {
            ROS_ERROR("Failed to approach picking table");
            break;
        }

        // Step 1b: Rotate to final picking orientation
        ros::Duration(0.5).sleep(); // Allow robot to settle

        move_base_msgs::MoveBaseGoal rotate_goal;
        rotate_goal.target_pose.header.frame_id = "odom";
        rotate_goal.target_pose.header.stamp = ros::Time::now();
        rotate_goal.target_pose.pose.position = approach_goal.target_pose.pose.position;

        tf2::Quaternion q_pick;
        q_pick.setRPY(0.0, 0.0, M_PI); // Face negative X direction
        rotate_goal.target_pose.pose.orientation = tf2::toMsg(q_pick);

        if (!navigateWithPrecision(ac, rotate_goal, 0.05, 0.087))
        {
            ROS_ERROR("Failed to align with picking table");
            break;
        }

        // -------------------------------------------------------------------------
        // PHASE 2: OBJECT DETECTION AND PICKING
        // -------------------------------------------------------------------------

        ROS_INFO("Phase 2: Object detection and picking...");

        // Step 2a: Position head for optimal table viewing
        control_msgs::FollowJointTrajectoryGoal head_goal;
        head_goal.trajectory.joint_names.push_back("head_1_joint");
        head_goal.trajectory.joint_names.push_back("head_2_joint");

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(-0.38); // Optimal head_1_joint position
        point.positions.push_back(-0.55); // Optimal head_2_joint position
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.time_from_start = ros::Duration(2.0);

        head_goal.trajectory.points.push_back(point);
        head_client.sendGoal(head_goal);
        head_client.waitForResult(ros::Duration(3.0));

        if (head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ros::Duration(1.0).sleep(); // Allow head to settle
        }

        // Step 2b: Get object pose from Node B
        assignment2_package::GetObjectPose get_pose_srv;
        if (!get_pose_client.call(get_pose_srv) || get_pose_srv.response.obj_id == -1)
        {
            // Fallback head position if initial detection fails
            ROS_WARN("Initial detection failed, trying fallback head position...");

            control_msgs::FollowJointTrajectoryGoal fallback_head_goal;
            fallback_head_goal.trajectory.joint_names.push_back("head_1_joint");
            fallback_head_goal.trajectory.joint_names.push_back("head_2_joint");

            trajectory_msgs::JointTrajectoryPoint fallback_point;
            fallback_point.positions.push_back(-0.0052); // Straight ahead
            fallback_point.positions.push_back(-0.7283); // Downward tilt
            fallback_point.velocities.push_back(0.0);
            fallback_point.velocities.push_back(0.0);
            fallback_point.time_from_start = ros::Duration(2.0);

            fallback_head_goal.trajectory.points.push_back(fallback_point);
            head_client.sendGoal(fallback_head_goal);
            head_client.waitForResult(ros::Duration(3.0));

            ros::Duration(1.0).sleep();

            // Retry object detection
            if (!get_pose_client.call(get_pose_srv) || get_pose_srv.response.obj_id == -1)
            {
                ROS_ERROR("Failed to detect objects after fallback. Aborting.");
                break;
            }
        }

        ROS_INFO("Detected object ID: %d", get_pose_srv.response.obj_id);

        // Step 2c: Call Node C to pick the object
        assignment2_package::PickObject pick_srv;
        pick_srv.request.target = get_pose_srv.response.obj_pose;
        pick_srv.request.target_id = get_pose_srv.response.obj_id;

        if (!pick_client.call(pick_srv) || !pick_srv.response.success)
        {
            ROS_ERROR("Failed to pick object %d", objects_processed + 1);
            
            // Return to safe position and try next object
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 9;
            goal.target_pose.pose.position.y = -1.0;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation = tf2::toMsg(q_safe);

            ac.sendGoal(goal);
            ac.waitForResult();
            ros::Duration(3.0).sleep(); // Allow objects to be re-detected
            continue;
        }

        ROS_INFO("Object picked successfully");
        ros::Duration(2.0).sleep(); // Allow planning scene to stabilize

        // -------------------------------------------------------------------------
        // PHASE 3: NAVIGATION TO PLACING TABLE
        // -------------------------------------------------------------------------

        ROS_INFO("Phase 3: Navigating to placing table...");

        // Step 3a: Rotate to face positive Y for straight-line navigation
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 8.881;
        goal.target_pose.pose.position.y = -3.02;
        goal.target_pose.pose.position.z = 0.0;

        tf2::Quaternion q_transit;
        q_transit.setRPY(0.0, 0.0, M_PI / 2); // Face positive Y direction
        goal.target_pose.pose.orientation = tf2::toMsg(q_transit);

        ac.sendGoal(goal);
        ac.waitForResult();

        // Step 3b: Navigate to placing table
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 9;
        goal.target_pose.pose.position.y = -1.92;
        goal.target_pose.pose.position.z = 0.0;

        tf2::Quaternion q_place;
        q_place.setRPY(0.0, 0.0, M_PI); // Face negative X direction
        goal.target_pose.pose.orientation = tf2::toMsg(q_place);

        if (!navigateWithPrecision(ac, goal, 0.03, 0.05))
        {
            ROS_ERROR("Failed to reach placing table");
            break;
        }

        ros::Duration(2.0).sleep(); // Allow robot to settle

        // -------------------------------------------------------------------------
        // PHASE 4: PLACEMENT POSE CALCULATION AND OBJECT PLACING
        // -------------------------------------------------------------------------

        ROS_INFO("Phase 4: Placement pose calculation and placing...");

        // Step 4a: Position head to see tag_10
        control_msgs::FollowJointTrajectoryGoal tag_head_goal;
        tag_head_goal.trajectory.joint_names.push_back("head_1_joint");
        tag_head_goal.trajectory.joint_names.push_back("head_2_joint");

        trajectory_msgs::JointTrajectoryPoint tag_point;
        tag_point.positions.push_back(0.176);  // Optimal head_1_joint for tag_10
        tag_point.positions.push_back(-0.710); // Optimal head_2_joint for tag_10
        tag_point.velocities.push_back(0.0);
        tag_point.velocities.push_back(0.0);
        tag_point.time_from_start = ros::Duration(2.0);

        tag_head_goal.trajectory.points.push_back(tag_point);
        head_client.sendGoal(tag_head_goal);
        head_client.waitForResult(ros::Duration(3.0));

        if (head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ros::Duration(1.0).sleep(); // Allow tag detection to stabilize
        }

        // Step 4b: Find optimal placement line (first visit only)
        if (!pose_manager.isInitialized())
        {
            ROS_INFO("First visit to placing table - finding optimal placement line...");
            ros::Duration(2.0).sleep(); // Ensure tag_10 is detected

            if (!pose_manager.findBestLineAndGeneratePoses(coeffs_client))
            {
                ROS_ERROR("Failed to find optimal placement line");
                break;
            }
        }
        else
        {
            double m, q;
            pose_manager.getLineInfo(m, q);
            ROS_INFO("Using cached placement line: y = %.3fx + %.3f", m, q);
        }

        // Step 4c: Get placement pose for current object
        geometry_msgs::PoseStamped placement_pose_base;
        if (!pose_manager.getPlacementPose(objects_processed, placement_pose_base, tfBuffer))
        {
            ROS_ERROR("Failed to get placement pose for object %d", objects_processed + 1);
            continue;
        }

        // Step 4d: Call Node C to place the object
        assignment2_package::PlaceObject place_srv;
        place_srv.request.target_id = get_pose_srv.response.obj_id;
        place_srv.request.target_pose = placement_pose_base;

        if (!place_client.call(place_srv) || !place_srv.response.success)
        {
            ROS_ERROR("Failed to place object %d", objects_processed + 1);
        }
        else
        {
            ROS_INFO("Object placed successfully");
        }

        // -------------------------------------------------------------------------
        // PREPARATION FOR NEXT ITERATION
        // -------------------------------------------------------------------------

        if (objects_processed < NUM_OBJECTS - 1)
        {
            ros::Duration(2.0).sleep(); // Allow planning scene to stabilize

            // Rotate to face negative Y for next picking approach
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 9;
            goal.target_pose.pose.position.y = -1.92;
            goal.target_pose.pose.position.z = 0.0;

            tf2::Quaternion q_return;
            q_return.setRPY(0.0, 0.0, -M_PI / 2); // Face negative Y direction
            goal.target_pose.pose.orientation = tf2::toMsg(q_return);

            ac.sendGoal(goal);
            ac.waitForResult();
        }

        objects_processed++;
        ROS_INFO("Completed object %d/%d", objects_processed, NUM_OBJECTS);
    }

    // =============================================================================
    // MISSION COMPLETION
    // =============================================================================

    ROS_INFO("Pick-and-place mission completed successfully!");
    ROS_INFO("Total objects processed: %d/%d", objects_processed, NUM_OBJECTS);
    
    ros::shutdown();
    return 0;
}