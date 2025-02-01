#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_objects");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Red Cube Collision Object
    moveit_msgs::CollisionObject red_cube;
    red_cube.header.frame_id = "base_link";  // Updated for consistency
    red_cube.id = "red_cube";

    // Define the cube as a box
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions = {0.06, 0.06, 0.06};

    // Pose of the red cube
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.9;

    red_cube.primitives.push_back(box);
    red_cube.primitive_poses.push_back(box_pose);
    red_cube.operation = moveit_msgs::CollisionObject::ADD;

    // Hexagonal Prism Collision Object
    moveit_msgs::CollisionObject hex_prism;
    hex_prism.header.frame_id = "base_link";
    hex_prism.id = "hexagonal_prism";

    // Define the cylinder
    shape_msgs::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    cylinder.dimensions = {0.12, 0.05};  // [height, radius]

    // Pose of the hexagonal prism
    geometry_msgs::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = 0.6;
    cylinder_pose.position.y = 0.4;
    cylinder_pose.position.z = 0.9;

    hex_prism.primitives.push_back(cylinder);
    hex_prism.primitive_poses.push_back(cylinder_pose);
    hex_prism.operation = moveit_msgs::CollisionObject::ADD;

    // Triangular Prism Collision Object
    moveit_msgs::CollisionObject tri_prism;
    tri_prism.header.frame_id = "base_link";
    tri_prism.id = "triangular_prism";

    // Define the shape as a box to approximate the triangular prism
    shape_msgs::SolidPrimitive tri_box;
    tri_box.type = shape_msgs::SolidPrimitive::BOX;
    tri_box.dimensions = {0.08, 0.08, 0.05};

    // Pose of the triangular prism
    geometry_msgs::Pose tri_pose;
    tri_pose.orientation.w = 1.0;
    tri_pose.position.x = 0.7;
    tri_pose.position.y = 0.2;
    tri_pose.position.z = 0.9;

    tri_prism.primitives.push_back(tri_box);
    tri_prism.primitive_poses.push_back(tri_pose);
    tri_prism.operation = moveit_msgs::CollisionObject::ADD;

    // Add collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(red_cube);
    collision_objects.push_back(hex_prism);
    collision_objects.push_back(tri_prism);

    // Add objects to the world and wait for them to be processed
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("Added collision objects, waiting for them to appear in Rviz...");
    ros::Duration(2.0).sleep();  // Allow time for processing

    ros::shutdown();
    return 0;
}
