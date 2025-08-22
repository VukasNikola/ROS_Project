/**
 * @file tables_node.cpp
 * @brief Static collision object manager for pick-and-place tables
 * 
 * This node creates and maintains static collision objects representing
 * the picking and placing tables in the MoveIt planning scene. These
 * collision objects ensure that:
 * - Path planning avoids colliding with the tables
 * - The robot can safely navigate around the workspace
 * - Manipulation planning accounts for table surfaces
 * 
 * Table Configuration:
 * - Both tables are identical box primitives (1.074m x 1.074m x 0.77m)
 * - Picking table: Located at (7.8, -3.02) in map frame
 * - Placing table: Located at (7.8, -1.92) in map frame
 * - Tables are positioned with their top surface at z=0.77m
 * 
 * Operation:
 * - Continuously publishes collision objects at 10Hz
 * - Updates timestamps to keep objects active in planning scene
 * - Ensures persistent collision avoidance throughout mission
 */

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief Main function - creates and maintains table collision objects
 * 
 * Workflow:
 * 1. Initialize ROS node and MoveIt planning scene interface
 * 2. Define box primitive for both tables (identical dimensions)
 * 3. Create collision objects for picking and placing tables
 * 4. Continuously update collision objects in planning scene
 * 
 * The continuous update loop ensures that the collision objects
 * remain active and are not automatically removed by MoveIt's
 * timeout mechanisms.
 */
int main(int argc, char **argv)
{
    // =============================================================================
    // INITIALIZATION
    // =============================================================================
    
    ros::init(argc, argv, "tables_node");
    ros::NodeHandle nh;
    
    // Use async spinner to handle callbacks while main thread runs collision updates
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Allow time for Gazebo and MoveIt to fully initialize
    ros::Duration(1.0).sleep();
    
    // Initialize MoveIt planning scene interface for collision object management
    moveit::planning_interface::PlanningSceneInterface psi;
    
    // =============================================================================
    // TABLE PRIMITIVE DEFINITION
    // =============================================================================
    
    // Define common box primitive for both tables
    // Dimensions match the actual table models in the simulation
    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {1.074, 1.074, 0.77}; // Length x Width x Height (meters)
    
    // Calculate Z position for table center (half of table height)
    // This positions the box so its top surface aligns with the table surface
    const double z_center = box.dimensions[2] / 2.0; // 0.385m
    
    // =============================================================================
    // PICKING TABLE COLLISION OBJECT
    // =============================================================================
    
    moveit_msgs::CollisionObject pick_table;
    pick_table.id = "picking_table";
    pick_table.header.frame_id = "map"; // Use map frame for global consistency
    pick_table.primitives.push_back(box);
    
    // Set picking table pose in map coordinates
    {
        geometry_msgs::Pose p;
        p.position.x = 7.8;   // X coordinate in map frame
        p.position.y = -3.02; // Y coordinate in map frame  
        p.position.z = z_center; // Center the box vertically (0.385m)
        p.orientation.w = 1.0;   // No rotation (identity quaternion)
        pick_table.primitive_poses.push_back(p);
    }
    
    // =============================================================================
    // PLACING TABLE COLLISION OBJECT
    // =============================================================================
    
    moveit_msgs::CollisionObject place_table;
    place_table.id = "placing_table";
    place_table.header.frame_id = "map"; // Use map frame for global consistency
    place_table.primitives.push_back(box);
    
    // Set placing table pose in map coordinates
    {
        geometry_msgs::Pose p;
        p.position.x = 7.8;   // X coordinate in map frame (same X as picking table)
        p.position.y = -1.92; // Y coordinate in map frame (1.1m offset from picking table)
        p.position.z = z_center; // Center the box vertically (0.385m)
        p.orientation.w = 1.0;   // No rotation (identity quaternion)
        place_table.primitive_poses.push_back(p);
    }
    
    // =============================================================================
    // COLLISION OBJECT MANAGEMENT LOOP
    // =============================================================================
    
    // Bundle both tables for efficient batch updates
    std::vector<moveit_msgs::CollisionObject> objects = {pick_table, place_table};
    
    // Set update frequency for collision object refresh
    ros::Rate rate(10.0); // 10 Hz - balances responsiveness with computational load
    
    ROS_INFO("Tables collision objects initialized. Publishing at 10Hz...");
    ROS_INFO("Picking table: (%.2f, %.2f, %.2f)", 7.8, -3.02, z_center);
    ROS_INFO("Placing table: (%.2f, %.2f, %.2f)", 7.8, -1.92, z_center);
    
    // Continuous update loop to maintain collision objects in planning scene
    while (ros::ok())
    {
        // Update timestamps to keep collision objects active
        // MoveIt may remove objects with stale timestamps
        ros::Time now = ros::Time::now();
        for (auto &obj : objects)
        {
            obj.header.stamp = now;
        }
        
        // Apply both collision objects to the planning scene
        // This ensures they remain available for path planning and collision checking
        psi.applyCollisionObjects(objects);
        
        // Sleep to maintain 10Hz update rate
        rate.sleep();
    }
    
    return 0;
}