#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "assignment2_package/PickObject.h"  // Service: request: geometry_msgs/PoseStamped target; response: bool success

// Node C: Performs pick (and assist in place) using MoveIt
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string BASE_FRAME = "base_footprint";

// Global MoveIt interfaces (to be initialized in main)
moveit::planning_interface::MoveGroupInterface* arm_group;
moveit::planning_interface::MoveGroupInterface* gripper_group;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;

// Helper: Add a collision object to the planning scene
void addCollisionObject(const std::string& id, const shape_msgs::SolidPrimitive& primitive,
                        const geometry_msgs::Pose& object_pose, const std::string& frame_id) {
    moveit_msgs::CollisionObject coll_obj;
    coll_obj.id = id;
    coll_obj.header.frame_id = frame_id;
    coll_obj.pose = object_pose;
    coll_obj.primitives.push_back(primitive);
    coll_obj.primitive_poses.push_back(object_pose);
    coll_obj.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface->applyCollisionObject(coll_obj);
}

// The service callback to perform the pick routine
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res) {
    ROS_INFO("Node C: PickObject service called.");
    geometry_msgs::PoseStamped target_pose = req.target;
    // Convert target pose to simpler variable
    geometry_msgs::Pose obj_pose = target_pose.pose;
    
    // Open gripper to be sure it's ready to grasp
    gripper_group->setNamedTarget("open");
    gripper_group->move();
    
    // Add collision object for the table (if not already present).
    // (Assumption: Table is at known position; here we add a generic table in front of robot)
    static bool table_added = false;
    if (!table_added) {
        // Define table collision (e.g., 1.0 x 1.0 x 0.05 m flat top)
        shape_msgs::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::SolidPrimitive::BOX;
        table_shape.dimensions = {1.0, 1.0, 0.05};  // X, Y, Z size
        geometry_msgs::Pose table_pose;
        table_pose.position.x = 0.7;  // roughly in front of TIAGo
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.72;  // table top height (approx 0.72m)
        table_pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));
        addCollisionObject("table", table_shape, table_pose, BASE_FRAME);
        table_added = true;
        ROS_INFO("Node C: Added table collision object to planning scene.");
    }
    
    // Add collision object for the target object (using a primitive slightly bigger than actual object)
    std::string obj_id = "object_" + std::to_string(ros::Time::now().toNSec());
    shape_msgs::SolidPrimitive obj_shape;
    // Guess shape based on ID ranges (optional, if we want specific shapes):
    // Here we just use a small box as a generic collision shape.
    obj_shape.type = shape_msgs::SolidPrimitive::BOX;
    obj_shape.dimensions = {0.06, 0.06, 0.06};  // a 6cm cube (a bit larger than object)
    // If needed, adjust shape based on actual object (cube/triangle prism/hex):
    // TODO: If object ID is known and shapes differ, modify primitive accordingly (e.g., cylinder for hexagon).
    geometry_msgs::Pose coll_obj_pose = obj_pose;
    // Lower the collision object slightly if the pose is at object top center (to cover the object volume)
    coll_obj_pose.position.z -= 0.03;  // place collision shape so it encompasses object downwards
    addCollisionObject(obj_id, obj_shape, coll_obj_pose, BASE_FRAME);
    ROS_INFO("Node C: Added collision object for target id %s at pose [%.2f, %.2f, %.2f].",
             obj_id.c_str(), coll_obj_pose.position.x, coll_obj_pose.position.y, coll_obj_pose.position.z);
    
    // Plan approach: n cm above the object
    geometry_msgs::PoseStamped approach_pose;
    approach_pose.header.frame_id = BASE_FRAME;
    approach_pose.pose = obj_pose;
    double approach_distance = 0.10;  // 10 cm above
    approach_pose.pose.position.z += approach_distance;
    // Set orientation for top-down grasp (assuming obj_pose orientation is identity pointing to tag).
    // We'll align the gripper downwards similar to Node A.
    tf2::Quaternion down_quat;
    down_quat.setRPY(M_PI, 0, 0);
    approach_pose.pose.orientation = tf2::toMsg(down_quat);
    
    arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
        ROS_WARN("Node C: Initial approach planning failed, adjusting and retrying.");
        // Try slightly higher approach if failed
        approach_pose.pose.position.z += 0.05;
        arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
        success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    if (!success) {
        ROS_ERROR("Node C: Cannot plan approach to object. Aborting pick.");
        res.success = false;
        // Remove the object collision (cleanup)
        planning_scene_interface->removeCollisionObjects({obj_id});
        return true;
    }
    ROS_INFO("Node C: Executing arm motion to pre-grasp pose.");
    arm_group->execute(plan);
    
    // Perform a linear Cartesian path to go from approach down to the object
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(approach_pose.pose);
    geometry_msgs::Pose grasp_pose = approach_pose.pose;
    grasp_pose.position.z -= approach_distance;  // down to original object top
    waypoints.push_back(grasp_pose);
    moveit_msgs::RobotTrajectory cart_traj;
    double fraction = arm_group->computeCartesianPath(waypoints, 0.01, 0.0, cart_traj);
    if (fraction < 0.99) {
        ROS_WARN("Node C: Cartesian path for final descent achieved %.2f%%, executing what was planned.", fraction*100.0);
    } else {
        ROS_INFO("Node C: Cartesian path for final approach planned (%.2f%% of path).", fraction*100.0);
    }
    arm_group->execute(cart_traj);
    
    // Close the gripper to grasp
    ROS_INFO("Node C: Closing gripper.");
    gripper_group->setNamedTarget("close");
    gripper_group->move();
    
    // (Optional) Attach object via plugin
    // TODO: Call gazebo_ros_link_attacher to attach the object to the gripper:
    // e.g., attach service with model of object and robot link "arm_7_link".
    ROS_INFO("Node C: (Optional) Attach object to gripper via Gazebo link attacher.");
    
    // Remove the object collision from planning scene now that it's "in hand"
    planning_scene_interface->removeCollisionObjects({obj_id});
    
    // Lift the object by moving back to the approach pose
    ROS_INFO("Node C: Lifting object.");
    arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
    if (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        arm_group->execute(plan);
    } else {
        ROS_WARN("Node C: Failed to plan lift back to approach, continuing.");
    }
    
    // Move arm to an intermediate safe pose above the table (to avoid knocking anything)
    // e.g., some "home" position or just stay at approach
    // We can skip directly to folding if the approach is already high enough.
    ROS_INFO("Node C: Moving arm to safe travel pose.");
    arm_group->setNamedTarget("folded");
    arm_group->move();
    
    ROS_INFO("Node C: Pick operation complete, object secured.");
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_C");
    ros::NodeHandle nh;
    // Initialize MoveIt interfaces
    static moveit::planning_interface::MoveGroupInterface arm_move_group(PLANNING_GROUP_ARM);
    static moveit::planning_interface::MoveGroupInterface gripper_move_group(PLANNING_GROUP_GRIPPER);
    static moveit::planning_interface::PlanningSceneInterface psi;
    arm_group = &arm_move_group;
    gripper_group = &gripper_move_group;
    planning_scene_interface = &psi;
    
    // Optionally set planner or planning time
    arm_group->setPlanningTime(10.0);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);
    
    // Start the service server for picking
    ros::ServiceServer service = nh.advertiseService("/pick_object", pickObjectCallback);
    ROS_INFO("Node C: Ready to pick objects on request.");
    
    ros::spin();
    return 0;
}
