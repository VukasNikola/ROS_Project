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
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "assignment2_package/GetObjectPose.h"
#include "assignment2_package/PickObject.h"

// Define some global parameters or constants
static const std::string PLANNING_GROUP_ARM = "arm";         // TIAGo arm planning group name
static const std::string PLANNING_GROUP_GRIPPER = "gripper"; // TIAGo gripper group (for opening/closing)
static const std::string BASE_FRAME = "base_footprint";      // robot base frame (or "base_link")
static const std::string TAG_FRAME = "tag_10";               // Reference frame of AprilTag ID 10 (placing table corner)

static const std::string PLANNING_GROUP_ARM_TORSO = "arm_torso"; // For reaching deep into a scene if needed

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Service client to get line coefficients from /straight_line_srv
    ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    tiago_iaslab_simulation::Coeffs coeffs_srv;
    float m = 0.0, q = 0.0;
    coeffs_srv.request.ready = true;
    if (coeffs_client.call(coeffs_srv))
    {
        m = coeffs_srv.response.coeffs[0];
        q = coeffs_srv.response.coeffs[1];
        ROS_INFO("Node A: Received line coefficients m=%.3f, q=%.3f from /straight_line_srv", m, q);
    }
    else
    {
        ROS_ERROR("Node A: Failed to call /straight_line_srv to get line coefficients");
        ros::shutdown();
        return 1;
    }

    // Initialize MoveIt interfaces for arm (for placing) and possibly gripper
    moveit::planning_interface::MoveGroupInterface arm_move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_move_group(PLANNING_GROUP_GRIPPER);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Allow some leeway in position/orientation (tolerances)
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
    ros::ServiceClient place_client = nh.serviceClient<assignment2_package::PickObject>("place_object");

    ROS_INFO("Waiting for services...");
    get_pose_client.waitForExistence();
    pick_client.waitForExistence(); 
    place_client.waitForExistence();
    ROS_INFO("All services available.");

    // Define placement points along the line y = m*x + q (in tag10 frame coordinates).
    int num_objects = 3; // we will place 3 objects (minimum required)
    double start_x = 0.20; // [m] starting x offset from tag frame origin (adjust as needed to be on table)
    double delta_x = 0.10; // [m] spacing in x between placements (adjust as needed)

    std::vector<geometry_msgs::PoseStamped> place_poses;
    place_poses.resize(num_objects);
    for (int i = 0; i < num_objects; ++i)
    {
        double x = start_x + i * delta_x;
        double y = m * x + q; // line equation
        // Set PoseStamped in tag10 frame
        place_poses[i].header.frame_id = TAG_FRAME;
        place_poses[i].pose.position.x = x;
        place_poses[i].pose.position.y = y;
        // Set Z position 0.6m above table for approach position
        place_poses[i].pose.position.z = 0.6; // 0.6m above table (Node C will move down from here)
        // Orientation: end-effector pointing downward
        tf2::Quaternion q_down;
        q_down.setRPY(M_PI, 0, 0); // 180 deg roll (flip), so that end-effector points down
        place_poses[i].pose.orientation = tf2::toMsg(q_down);
    }

    // TF listener to get transforms (for converting poses from tag frame to base frame)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(2.0).sleep(); // wait for TF frames to become available

    // Initial navigation to avoid table collision
    ROS_INFO("Node A: Moving to initial safe position to avoid table collision...");
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 9;
    goal.target_pose.pose.position.y = -1.0;  // Safe Y position
    goal.target_pose.pose.position.z = 0.0;
    {
        tf2::Quaternion q;
        double yaw = -M_PI/2; // Facing negative Y direction
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
        goal.target_pose.pose.position.x = 9;  // Leave room for head movement
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

        // STEP 2: Get object pose from Node B
        ROS_INFO("Node A: Getting object pose from Node B...");
        assignment2_package::GetObjectPose get_pose_srv;
        if (!get_pose_client.call(get_pose_srv))
        {
            ROS_ERROR("Node A: Failed to call get_object_pose service");
            break;
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
                double yaw = -M_PI/2; // Facing negative Y direction
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

        // STEP 4: Navigate to placing table
        ROS_INFO("Node A: Navigating to placing table...");
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 9;  // Leave room for arm movement  
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

        // STEP 5: Move arm to approach pose above placement line
        geometry_msgs::PoseStamped place_pose_tag = place_poses[objects_processed];
        geometry_msgs::PoseStamped place_pose_base;
        try
        {
            // Transform the tag10 frame placement pose to base frame
            place_pose_tag.header.stamp = ros::Time(0);
            tfBuffer.transform(place_pose_tag, place_pose_base, BASE_FRAME);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Node A: TF transform error from tag frame to base frame: %s", ex.what());
            continue; // Skip this object
        }
        
        ROS_INFO("Node A: Moving arm to approach pose above placement line...");
        arm_move_group.setPoseTarget(place_pose_base.pose, "arm_tool_link");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (!success)
        {
            ROS_ERROR("Node A: Failed to plan approach to place position. Skipping object %d.", objects_processed + 1);
            continue;
        }
        
        ROS_INFO("Node A: Executing approach to place position...");
        arm_move_group.execute(plan);
        ros::Duration(1.0).sleep(); // Allow settling

        // STEP 6: Call Node C to place the object
        ROS_INFO("Node A: Calling Node C to place object...");
        assignment2_package::PickObject place_srv; // Reusing PickObject service type
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

        // STEP 7: Move arm to safe travel position
        ROS_INFO("Node A: Moving arm to safe travel position...");
        try 
        {
            arm_move_group.setNamedTarget("folded"); // Assuming "folded" pose exists
            arm_move_group.move();
        }
        catch (...)
        {
            ROS_WARN("Node A: Failed to move to 'folded' position. Arm may not have a predefined folded pose.");
        }

        objects_processed++;
        ROS_INFO("Node A: Completed pick-and-place for object %d/%d", objects_processed, num_objects);
    }

    ROS_INFO("Node A: Pick-and-place routine completed. Processed %d objects.", objects_processed);
    ros::shutdown();
    return 0;
}