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

// ==================== PLACEMENT POSE MANAGER CLASS ====================
class PlacementPoseManager
{
private:
    // Store the line coefficients and table info
    double line_m_;
    double line_q_;
    double table_center_x_;
    double table_center_y_;

    // Store the placement points in tag_10 frame
    std::vector<std::pair<double, double>> placement_points_tag_frame_;
    bool poses_initialized_;

    // Table dimensions (matching your test node)
    static constexpr double TABLE_WIDTH = 1.07;
    static constexpr double TABLE_DEPTH = 1.07;
    static constexpr double TABLE_HEIGHT = 0.77;
    static constexpr double TABLE_OFFSET_X = 0.36;
    static constexpr double TABLE_OFFSET_Y = 0.145;
    static constexpr double EPSILON = 1e-6;

public:
    PlacementPoseManager() : poses_initialized_(false) {}

    /**
     * @brief Check if a point is within the table bounds
     */
    bool isPointOnTable(double x, double y)
    {
        double x_min = table_center_x_ - TABLE_WIDTH / 2.0;
        double x_max = table_center_x_ + TABLE_WIDTH / 2.0;
        double y_min = table_center_y_ - TABLE_DEPTH / 2.0;
        double y_max = table_center_y_ + TABLE_DEPTH / 2.0;

        return (x >= x_min - EPSILON && x <= x_max + EPSILON &&
                y >= y_min - EPSILON && y <= y_max + EPSILON);
    }

    /**
     * @brief Initialize placement poses along the line in tag_10 frame
     * This matches the logic from your test node
     */
    bool initializePoses(double line_m, double line_q, int num_objects = 3)
    {
        ROS_INFO("Initializing %d placement poses along line y = %.3fx + %.3f", num_objects, line_m, line_q);

        // Store line coefficients
        line_m_ = line_m;
        line_q_ = line_q;

        // Set table center (matching your test node)
        table_center_x_ = TABLE_OFFSET_X;
        table_center_y_ = TABLE_OFFSET_Y;

        // Clear any existing points
        placement_points_tag_frame_.clear();

        // Find the closest point on the line to the table center
        double closest_x, closest_y;

        if (std::abs(line_m) < EPSILON)
        {
            // Horizontal line: y = q
            closest_x = table_center_x_;
            closest_y = line_q;
        }
        else if (std::abs(1.0 / line_m) < EPSILON)
        {
            // Vertical line: x = -q/m
            closest_x = -line_q / line_m;
            closest_y = table_center_y_;
        }
        else
        {
            // General case: find perpendicular intersection
            double m_perp = -1.0 / line_m;
            double q_perp = table_center_y_ - m_perp * table_center_x_;
            closest_x = (q_perp - line_q) / (line_m - m_perp);
            closest_y = line_m * closest_x + line_q;
        }

        // Calculate direction vector along the line
        double dir_x, dir_y;
        if (std::abs(line_m) < EPSILON)
        {
            // Horizontal line
            dir_x = 1.0;
            dir_y = 0.0;
        }
        else if (std::abs(1.0 / line_m) < EPSILON)
        {
            // Vertical line
            dir_x = 0.0;
            dir_y = 1.0;
        }
        else
        {
            // General case
            dir_x = 1.0;
            dir_y = line_m;
            double length = std::sqrt(dir_x * dir_x + dir_y * dir_y);
            dir_x /= length;
            dir_y /= length;
        }

        // Generate placement points: center point ± 0.16m along the line
        double spacing = 0.16;

        // Center point (closest to table center)
        placement_points_tag_frame_.push_back({closest_x, closest_y});

        // Point 0.15m "above" (in positive direction along line)
        placement_points_tag_frame_.push_back({closest_x + spacing * dir_x,
                                               closest_y + spacing * dir_y});

        // Point 0.15m "below" (in negative direction along line)
        placement_points_tag_frame_.push_back({closest_x - spacing * dir_x,
                                               closest_y - spacing * dir_y});

        // Verify all points are on the table
        bool all_valid = true;
        for (size_t i = 0; i < placement_points_tag_frame_.size(); ++i)
        {
            double x = placement_points_tag_frame_[i].first;
            double y = placement_points_tag_frame_[i].second;

            if (!isPointOnTable(x, y))
            {
                ROS_WARN("Placement point %zu at (%.3f, %.3f) is off the table!", i, x, y);
                all_valid = false;
            }
            else
            {
                std::string position_name = (i == 0) ? "CENTER" : (i == 1 ? "FORWARD" : "BACKWARD");
                ROS_INFO("Placement %zu (%s): tag_10(%.3f, %.3f)", i, position_name.c_str(), x, y);
            }
        }

        if (!all_valid)
        {
            ROS_ERROR("Some placement points are off the table. Line may not be suitable.");
            // Continue anyway, but warn the user
        }

        poses_initialized_ = true;
        ROS_INFO("Successfully initialized %zu placement poses", placement_points_tag_frame_.size());
        return true;
    }

    /**
     * @brief Get a placement pose transformed to base_footprint frame
     * Call this right before you need to place an object
     * @param object_index Index of the object (0, 1, 2, ...)
     * @param pose_out Output pose in base_footprint frame
     * @param tfBuffer TF buffer for transformations
     * @param z_offset Height above table surface (default 0.3m above table to match Node C)
     */
    bool getPlacementPose(int object_index, geometry_msgs::PoseStamped &pose_out,
                          tf2_ros::Buffer &tfBuffer, double z_offset = 0.3)
    {
        if (!poses_initialized_)
        {
            ROS_ERROR("Poses not initialized. Call initializePoses() first.");
            return false;
        }

        if (object_index < 0 || object_index >= placement_points_tag_frame_.size())
        {
            ROS_ERROR("Invalid object index %d. Valid range: 0-%zu",
                      object_index, placement_points_tag_frame_.size() - 1);
            return false;
        }

        try
        {
            // Use SINGLE timestamp for consistency
            ros::Time current_time = ros::Time::now();

            // Get the stored placement point in tag_10 frame
            double x = placement_points_tag_frame_[object_index].first;
            double y = placement_points_tag_frame_[object_index].second;

            geometry_msgs::PoseStamped pose_tag;
            pose_tag.header.frame_id = TAG_FRAME;
            pose_tag.header.stamp = current_time;
            pose_tag.pose.position.x = x;
            pose_tag.pose.position.y = y;
            pose_tag.pose.position.z = 0.0;

            // End-effector pointing downward
            tf2::Quaternion q_down;
            q_down.setRPY(M_PI, 0, 0);
            pose_tag.pose.orientation = tf2::toMsg(q_down);

            // *** ADD GRIPPER OFFSET LIKE IN PICKING ***
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

            // Transform to base_footprint
            tfBuffer.transform(pose_tag, pose_out, BASE_FRAME, ros::Duration(3.0));

            // Set approach height: table surface (0.77m) + z_offset
            pose_out.pose.position.z = TABLE_HEIGHT + z_offset;

            // Use torso_fixed_link orientation for all placements
            ROS_INFO("Using torso_fixed_link orientation for placement (object orientation doesn't matter)");
            geometry_msgs::TransformStamped torso_transform;
            try
            {
                torso_transform = tfBuffer.lookupTransform(BASE_FRAME, "torso_fixed_link", ros::Time(0), ros::Duration(1.0));
                tf2::Quaternion torso_q;
                tf2::fromMsg(torso_transform.transform.rotation, torso_q);

                // Create 180 degree rotation around X
                tf2::Quaternion x_rotation;
                x_rotation.setRPY(M_PI, 0, 0);

                // Apply the rotation
                tf2::Quaternion final_q = torso_q * x_rotation;
                pose_out.pose.orientation = tf2::toMsg(final_q);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("Could not get torso_fixed_link transform: %s", ex.what());
                return false;
            }

            std::string position_name = (object_index == 0) ? "CENTER" : (object_index == 1 ? "FORWARD" : "BACKWARD");
            ROS_INFO("Approach pose %d (%s): tag_10(%.3f, %.3f) -> base(%.3f, %.3f, %.3f) [+%.2fm above table, with gripper offset]",
                     object_index, position_name.c_str(),
                     x, y,
                     pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z,
                     z_offset);

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("TF transform error for object %d: %s", object_index, ex.what());
            return false;
        }
    }

    /**
     * @brief Get number of stored poses
     */
    int getNumPoses() const
    {
        return poses_initialized_ ? placement_points_tag_frame_.size() : 0;
    }

    /**
     * @brief Check if poses are initialized
     */
    bool isInitialized() const
    {
        return poses_initialized_;
    }

    /**
     * @brief Clear stored poses
     */
    void clear()
    {
        placement_points_tag_frame_.clear();
        poses_initialized_ = false;
        ROS_INFO("Cleared stored placement poses");
    }

    /**
     * @brief Get stored line info
     */
    void getLineInfo(double &m, double &q) const
    {
        m = line_m_;
        q = line_q_;
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

    // ==================== INITIALIZE PLACEMENT POSE MANAGER ====================
    PlacementPoseManager pose_manager;
    int num_objects = 3;               // we will place 3 objects (minimum required)
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

    // Set tolerances for arm_torso group (matching Node C)
    arm_torso_move_group.setGoalPositionTolerance(0.01);
    arm_torso_move_group.setGoalOrientationTolerance(0.01);
    arm_torso_move_group.setMaxAccelerationScalingFactor(0.5);
    arm_torso_move_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_move_group.setPlanningTime(10.0);

    // Also set for arm group
    arm_move_group.setGoalPositionTolerance(0.01);
    arm_move_group.setGoalOrientationTolerance(0.01);
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
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 8.88;  // Leave room for head movement
        goal.target_pose.pose.position.y = -3.02; // Correct Y for picking table
        goal.target_pose.pose.position.z = 0.0;
        {
            tf2::Quaternion q;
            double yaw = M_PI; // Facing negative X direction
            q.setRPY(0.0, 0.0, yaw);
            goal.target_pose.pose.orientation = tf2::toMsg(q);
        }

        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Failed to reach picking table. Aborting.");
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

        // STEP 3.6: Move arm to safe travel position after clearing table
        ROS_INFO("Node A: Moving arm to safe position before next pick cycle...");
        try
        {
            std::map<std::string, double> safe_travel_joints;
            safe_travel_joints["torso_lift_joint"] = 0.150;
            safe_travel_joints["arm_1_joint"] = 0.200;
            safe_travel_joints["arm_2_joint"] = -1.339;
            safe_travel_joints["arm_3_joint"] = -0.200;
            safe_travel_joints["arm_4_joint"] = 1.938;
            safe_travel_joints["arm_5_joint"] = -1.570;
            safe_travel_joints["arm_6_joint"] = 1.370;
            safe_travel_joints["arm_7_joint"] = 0.00;

            arm_torso_move_group.setJointValueTarget(safe_travel_joints);

            // Increase planning time and attempts
            arm_torso_move_group.setPlanningTime(10.0);
            arm_torso_move_group.setNumPlanningAttempts(5);

            // Plan first, then execute
            moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
            bool plan_success = (arm_torso_move_group.plan(safe_plan) ==
                                 moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (plan_success)
            {
                // Slow down for safe movement
                arm_torso_move_group.setMaxVelocityScalingFactor(0.3);
                arm_torso_move_group.setMaxAccelerationScalingFactor(0.3);

                moveit::core::MoveItErrorCode exec_result = arm_torso_move_group.execute(safe_plan);

                // Reset scaling
                arm_torso_move_group.setMaxVelocityScalingFactor(0.5);
                arm_torso_move_group.setMaxAccelerationScalingFactor(0.5);

                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Node A: Arm moved to safe position.");
                    ros::Duration(1.0).sleep();
                }
                else
                {
                    ROS_WARN("Node A: Failed to execute safe position movement (error %d), continuing anyway.",
                             exec_result.val);
                }
            }
            else
            {
                ROS_WARN("Node A: Failed to plan safe position movement, continuing anyway.");
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Node A: Exception moving arm to safe position: %s, continuing.", e.what());
        }

        // NEW STEP 3.7: Rotate to face positive Y direction to avoid arc navigation
        ROS_INFO("Node A: Rotating to face positive Y direction for straight navigation...");
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 9;     // Keep same position
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

        // STEP 4: Navigate to placing table
        ROS_INFO("Node A: Navigating to placing table...");
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 8.97;
        goal.target_pose.pose.position.y = -1.92; // Correct Y for placing table
        goal.target_pose.pose.position.z = 0.0;
        {
            tf2::Quaternion q;
            double yaw = M_PI; // Facing negative X direction
            q.setRPY(0.0, 0.0, yaw);
            goal.target_pose.pose.orientation = tf2::toMsg(q);
        }

        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Failed to reach placing table. Aborting.");
            break;
        }
        ROS_INFO("Successfully reached placing table.");

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
        if (!best_line_found)
        {
            ROS_INFO("Node A: First time at placing table - finding best line from 300 iterations...");

            // Wait a bit more to ensure tag_10 is being detected
            ros::Duration(2.0).sleep();

            struct LineCandidate
            {
                double m, q;
                double distance_to_center;
                bool all_points_valid;
            };

            std::vector<LineCandidate> valid_lines;
            const double TABLE_CENTER_X = 0.36;  // Matching test node
            const double TABLE_CENTER_Y = 0.145; // Matching test node

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

                ROS_INFO("Iteration %d: Line y = %.3fx + %.3f", iteration, m, q);

                // Calculate distance to table center (simplified version)
                double closest_x, closest_y;
                if (std::abs(m) < 1e-6)
                {
                    closest_x = TABLE_CENTER_X;
                    closest_y = q;
                }
                else
                {
                    double m_perp = -1.0 / m;
                    double q_perp = TABLE_CENTER_Y - m_perp * TABLE_CENTER_X;
                    closest_x = (q_perp - q) / (m - m_perp);
                    closest_y = m * closest_x + q;
                }

                double distance = std::sqrt(std::pow(closest_x - TABLE_CENTER_X, 2) +
                                            std::pow(closest_y - TABLE_CENTER_Y, 2));

                LineCandidate candidate;
                candidate.m = m;
                candidate.q = q;
                candidate.distance_to_center = distance;
                candidate.all_points_valid = true;

                valid_lines.push_back(candidate);
                ROS_INFO("  - Distance to table center: %.4f", distance);
            }

            if (valid_lines.empty())
            {
                ROS_ERROR("No valid lines found after 300 iterations");
                break; // Skip this object
            }

            // Find the best line (closest to table center)
            auto best_line = std::min_element(valid_lines.begin(), valid_lines.end(),
                                              [](const LineCandidate &a, const LineCandidate &b)
                                              {
                                                  return a.distance_to_center < b.distance_to_center;
                                              });

            best_m = best_line->m;
            best_q = best_line->q;

            ROS_INFO("===== BEST LINE SELECTED =====");
            ROS_INFO("Line: y = %.3fx + %.3f", best_m, best_q);
            ROS_INFO("Distance to table center: %.4f", best_line->distance_to_center);
            ROS_INFO("This line will be used for ALL placements");
            ROS_INFO("==============================");

            // Initialize placement poses with the best line
            if (!pose_manager.initializePoses(best_m, best_q, num_objects))
            {
                ROS_ERROR("Node A: Failed to initialize placement poses");
                break; // Skip this object
            }

            best_line_found = true;
            ROS_INFO("Node A: Placement poses initialized for %d objects using best line", num_objects);
        }
        else
        {
            ROS_INFO("Node A: Using previously found best line y = %.3fx + %.3f", best_m, best_q);
        }

        // STEP 5: Get placement pose and move to approach position using arm_torso
        ROS_INFO("Node A: Getting approach pose for object %d (0.3m above placement)...", objects_processed + 1);

        // IMPORTANT: Update poses each time we return to the placing table
        // This ensures the transformation from tag_10 to base_footprint is current
        ROS_INFO("Node A: Updating placement pose transformation for current robot position...");

        geometry_msgs::PoseStamped approach_pose_base;

        if (!pose_manager.getPlacementPose(objects_processed, approach_pose_base, tfBuffer, 0.3))
        {
            ROS_ERROR("Node A: Failed to get approach pose for object %d. Skipping.", objects_processed + 1);
            continue; // Skip this object
        }

        // The pose is now freshly transformed from tag_10 to base_footprint
        // using the current TF, so it will be at the correct position
        ROS_INFO("Node A: Placement pose updated with current transform");

        // IMPORTANT: Use arm_torso for better reach to placement position
        ROS_INFO("Node A: Moving arm to approach pose above placement position using arm_torso...");
        arm_torso_move_group.setEndEffectorLink("gripper_grasping_frame"); // Match Node C
        arm_torso_move_group.setPoseTarget(approach_pose_base.pose, "gripper_grasping_frame");
        arm_torso_move_group.setPlanningTime(30.0);
        arm_torso_move_group.setNumPlanningAttempts(20);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_torso_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            ROS_ERROR("Node A: Failed to plan approach to placement position with arm_torso. Trying alternative approach...");

            // Try with arm_tool_link as end effector
            arm_torso_move_group.setEndEffectorLink("arm_tool_link");
            arm_torso_move_group.setPoseTarget(approach_pose_base.pose, "arm_tool_link");
            success = (arm_torso_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (!success)
            {
                ROS_ERROR("Node A: Still failed to plan. Skipping object %d.", objects_processed + 1);
                continue;
            }
        }

        ROS_INFO("Node A: Executing approach to placement position...");
        arm_torso_move_group.execute(plan);
        ros::Duration(2.0).sleep(); // Allow settling

        // STEP 6: Call Node C to place the object (it will handle the final positioning)
        ROS_INFO("Node A: Calling Node C to place object from approach pose...");
        assignment2_package::PlaceObject place_srv;
        place_srv.request.target_id = get_pose_srv.response.obj_id;

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
            // STEP 6.6: Move arm to safe position
            ROS_INFO("Node A: Moving arm to safe position before next pick cycle...");
            try
            {
                std::map<std::string, double> safe_travel_joints;
                safe_travel_joints["torso_lift_joint"] = 0.150;
                safe_travel_joints["arm_1_joint"] = 0.200;
                safe_travel_joints["arm_2_joint"] = -1.339;
                safe_travel_joints["arm_3_joint"] = -0.200;
                safe_travel_joints["arm_4_joint"] = 1.938;
                safe_travel_joints["arm_5_joint"] = -1.570;
                safe_travel_joints["arm_6_joint"] = 1.370;
                safe_travel_joints["arm_7_joint"] = 0.00;

                arm_torso_move_group.setJointValueTarget(safe_travel_joints);

                // Increase planning time and attempts
                arm_torso_move_group.setPlanningTime(15.0);
                arm_torso_move_group.setNumPlanningAttempts(10);

                // Plan first, then execute
                moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
                bool plan_success = (arm_torso_move_group.plan(safe_plan) ==
                                     moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (plan_success)
                {
                    // Slow down for safe movement
                    arm_torso_move_group.setMaxVelocityScalingFactor(0.3);
                    arm_torso_move_group.setMaxAccelerationScalingFactor(0.3);

                    moveit::core::MoveItErrorCode exec_result = arm_torso_move_group.execute(safe_plan);

                    // Reset scaling
                    arm_torso_move_group.setMaxVelocityScalingFactor(0.5);
                    arm_torso_move_group.setMaxAccelerationScalingFactor(0.5);

                    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        ROS_INFO("Node A: Arm moved to safe position.");
                        ros::Duration(1.0).sleep();
                    }
                    else
                    {
                        ROS_WARN("Node A: Failed to execute safe position movement (error %d), continuing anyway.",
                                 exec_result.val);
                    }
                }
                else
                {
                    ROS_WARN("Node A: Failed to plan safe position movement, continuing anyway.");
                }
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Node A: Exception moving arm to safe position: %s, continuing.", e.what());
            }

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