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

    // === Define an explicit “open” joint configuration ===
    std::map<std::string, double> open_position;
    open_position["gripper_left_finger_joint"] = 0.04;
    open_position["gripper_right_finger_joint"] = 0.04;

    // Open gripper
    gripper_move_group.setJointValueTarget(open_position);
    if (gripper_move_group.move())
        ROS_INFO("Node A: Gripper opened.");
    else
        ROS_WARN("Node A: Gripper open failed.");
    
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base action server.");

    // Create an action client for move_base.
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "odom";

    // --- Navigation Sequence to the Pick-Up Table ---
    // 1st Goal: (8.9, -1) with yaw = 0.
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 8.9;
    goal.target_pose.pose.position.y = -1.0;
    {
        tf2::Quaternion q;
        double yaw = 0.0; // facing positive x
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);
    }
    ROS_INFO("Sending 1st goal: (8.9, -1) with yaw = 0");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach first goal.");
        return 1;
    }
    ROS_INFO("Reached first goal.");

    // 2nd Goal: (8.9, -2.5) with yaw = 0.
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 8.9;
    goal.target_pose.pose.position.y = -2.5;
    {
        tf2::Quaternion q;
         double yaw = M_PI;
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);
    }
    ROS_INFO("Sending 2nd goal: (8.9, -2.5) with yaw = 0");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach second goal.");
        return 1;
    }
    ROS_INFO("Reached second goal.");

    // Define placement points along the line y = m*x + q (in tag10 frame coordinates).
    // We choose 3 points with some fixed spacing in x. These should lie within the table surface.
    int num_objects = 3; // we will place 3 objects (minimum required)

    // NEEDS ADJUSTING

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
        // Set Z position slightly above table: assume tag frame is at table surface, so z = object height/2 + clearance.
        place_poses[i].pose.position.z = 0.05; // 5cm above table (to place at surface, we will later approach down)
        // Orientation: we want the end-effector to be pointing downward.
        // We set orientation such that gripper's tool frame has z-axis pointing down.
        // (This is an approximate orientation; adjust via RPY as needed.)
        tf2::Quaternion q_down;
        q_down.setRPY(M_PI, 0, 0); // 180 deg roll (flip), so that end-effector points down (assuming default orientation facing forward)
        place_poses[i].pose.orientation = tf2::toMsg(q_down);
    }

    // TF listener to get transforms (for converting poses from tag frame to base frame)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(1.0).sleep(); // wait a moment for TF frames to become available

    // Loop over each object to pick and place
   

    for (int idx = 0; idx < num_objects && ros::ok(); ++idx)
    {
        ROS_INFO("Node A: Starting pick-and-place for object %d", idx + 1);

        // 3rd Goal: Adjust to (8.7, -2.5) with yaw = π (robot faces negative x).
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 8.7;
        goal.target_pose.pose.position.y = -2.5;
        {
            tf2::Quaternion q;
            double yaw = M_PI;
            q.setRPY(0.0, 0.0, yaw);
            goal.target_pose.pose.orientation = tf2::toMsg(q);
        }

        ROS_INFO("Sending 3rd goal: (8.7, -2.5) with yaw = π");
        ac.sendGoal(goal);
        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Failed to reach third goal.");
            return 1;
        }
        ROS_INFO("Successfully reached the pick-up table.");

        // ** Get target object pose from Node B (detection) **
        // We assume Node B offers a service "/get_object_pose" that returns the next object's pose in base frame.
        // Alternatively, Node A could call Node B to get all poses and select one.
        geometry_msgs::PoseStamped object_pose_base;
        {
            // Service call to Node B - GetObjectPose (assuming custom service that returns PoseStamped of an unpicked object)
            // For demonstration, we will call it and expect object_pose_base in response.
            // If Node B automatically transforms to base frame, great; otherwise, it might return in camera or tag frame.
            // Here we assume it returns pose in the robot base frame.
            assignment2_package::GetObjectPose getObjSrv;
            if (!ros::service::call("/get_object_pose", getObjSrv))
            {
                ROS_ERROR("Node A: Failed to call Node B service to get object pose");
                break;
            }
            object_pose_base = getObjSrv.response.obj_pose;
        }
        ROS_INFO_STREAM("Node A: Received object pose from Node B: " << object_pose_base);

        // ** Call Node C to pick the object at the given pose **
        // We assume Node C provides a service "/pick_object" that takes a PoseStamped (in base frame) and picks the object.
        // The service could return success or failure.
        {
            assignment2_package::PickObject pickSrv;
            pickSrv.request.target = object_pose_base;
            ROS_INFO("Node A: Calling Node C to pick the object...");
            if (!ros::service::call("/pick_object", pickSrv) || !pickSrv.response.success)
            {
                ROS_ERROR("Node A: Node C failed to pick object %d. Aborting sequence.", idx + 1);
                break;
            }
        }
        ROS_INFO("Node A: Object %d picked successfully, now navigating to place table.", idx + 1);

        // ** Navigate to placing table position **
        // TODO: Command robot base to move to the placing table docking position.
        // e.g., call a navigation service or action to go to a preset pose in front of the placing table.
        ROS_INFO("Node A: (TODO) Navigate to placing table pose");

        // ** Compute and execute place motion using MoveIt **
        // Take the pre-defined placement pose in tag frame and transform it to base frame for current robot pose.
        geometry_msgs::PoseStamped place_pose_tag = place_poses[idx];
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
            // If transform fails, skip this iteration
            continue;
        }
        ROS_INFO_STREAM("Node A: Transformed place pose to base frame: " << place_pose_base);

        // Plan and execute a movement to an approach pose above the place location.
        // We will approach from above (already set orientation downwards in place_pose_base).
        geometry_msgs::PoseStamped approach_pose = place_pose_base;
        approach_pose.pose.position.z += 0.10; // approach 10cm above the table target point
        arm_move_group.setPoseTarget(approach_pose.pose, "arm_tool_link");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ROS_WARN("Node A: Failed to plan approach to place position. Retrying with a lower height.");
            // Try a slightly higher approach if initial fails (simple retry strategy)
            approach_pose.pose.position.z += 0.05;
            arm_move_group.setPoseTarget(approach_pose.pose, "arm_tool_link");
            success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
        if (success)
        {
            ROS_INFO("Node A: Executing approach to place position...");
            arm_move_group.execute(plan);
            ros::Duration(1.0).sleep(); // small delay
        }
        else
        {
            ROS_ERROR("Node A: Failed to plan any trajectory to place approach. Skipping placement for object %d.", idx + 1);
            continue;
        }

        // Now perform a *linear* motion downwards from the approach to the actual place pose.
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(approach_pose.pose);
        waypoints.push_back(place_pose_base.pose);
        moveit_msgs::RobotTrajectory cart_traj;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01; // e.g., 1cm step for Cartesian path
        double frac = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cart_traj);
        if (frac > 0.99)
        {
            ROS_INFO("Node A: Cartesian path computed for final lowering (%.2f%% achieved).", frac * 100.0);
            // Execute the Cartesian trajectory for placing down
            arm_move_group.execute(cart_traj);
        }
        else
        {
            ROS_WARN("Node A: Cartesian path for placing only achieved %.2f%%, trying to execute partial path.", frac * 100.0);
            arm_move_group.execute(cart_traj);
        }

        // Once at the place pose (object over the table), open the gripper to release the object
        ROS_INFO("Node A: Opening gripper to release object.");
        gripper_move_group.setNamedTarget("open");
        gripper_move_group.move();

        // (Optional) Detach object via Gazebo_ros_link_attacher plugin
        // TODO: Call the appropriate service to detach the object from the gripper, if it was attached.
        // Example (if a service /link_attacher_node/detach is available):
        // gazebo_ros_link_attacher::Attach detach_srv;
        // detach_srv.request.model_name_1 = "tiago"; detach_srv.request.link_name_1 = "arm_7_link";
        // detach_srv.request.model_name_2 = "<object_model_name>"; detach_srv.request.link_name_2 = "<object_link_name>";
        // ros::service::call("/link_attacher_node/detach", detach_srv);
        ROS_INFO("Node A: (Optional) Detach object via Gazebo link attacher plugin (if in use).");

        // Retreat arm back to the approach pose (or intermediate safe pose)
        arm_move_group.setPoseTarget(approach_pose.pose, "arm_tool_link");
        if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            arm_move_group.execute(plan);
            ros::Duration(0.5).sleep();
        }
        // Move arm to a secure travel pose (folded arm) before navigating again
        arm_move_group.setNamedTarget("folded"); // assuming a named pose "folded" or "home" is defined for safe travel
        arm_move_group.move();
        ROS_INFO("Node A: Placed object %d and moved arm to safe position.", idx + 1);

        // After placing, go back for next object (loop continues)
    } // end for each object

    ROS_INFO("Node A: Pick-and-place routine completed.");
    ros::shutdown();
    return 0;
}
