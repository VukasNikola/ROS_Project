#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include "tiago_iaslab_simulation/Coeffs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <std_msgs/ColorRGBA.h>

static const std::string TAG_FRAME = "tag_10";
static const std::string BASE_FRAME = "base_footprint";
static const double PLACEMENT_Z_HEIGHT = 0.45; // Fixed Z height for placement markers in BASE_FRAME
static const double TABLE_WIDTH = 1.07; // Assuming a square table for simplicity.
static const double TABLE_DEPTH = 1.07; // Assuming a square table for simplicity.
static const double TABLE_HEIGHT = 0.77; // Table thickness

/**
 * @brief Finds the intersection points of a line and a rectangle.
 * @param m The slope of the line (y = mx + q).
 * @param q The y-intercept of the line.
 * @param x_min The minimum x-coordinate of the rectangle.
 * @param x_max The maximum x-coordinate of the rectangle.
 * @param y_min The minimum y-coordinate of the rectangle.
 * @param y_max The maximum y-coordinate of the rectangle.
 * @return A vector of pairs representing the (x, y) intersection points.
 */
std::vector<std::pair<double, double>> findLineTableIntersections(
    double m, double q, 
    double x_min, double x_max, 
    double y_min, double y_max) 
{
    std::vector<std::pair<double, double>> intersections;
    const double epsilon = 1e-6;
    
    // Check intersection with left edge (x = x_min)
    double y_left = m * x_min + q;
    if (y_left >= y_min - epsilon && y_left <= y_max + epsilon) {
        intersections.push_back({x_min, y_left});
    }
    
    // Check intersection with right edge (x = x_max)
    double y_right = m * x_max + q;
    if (y_right >= y_min - epsilon && y_right <= y_max + epsilon) {
        intersections.push_back({x_max, y_right});
    }
    
    // Check intersection with bottom edge (y = y_min)
    if (std::abs(m) > epsilon) { 
        double x_bottom = (y_min - q) / m;
        if (x_bottom >= x_min - epsilon && x_bottom <= x_max + epsilon) {
            intersections.push_back({x_bottom, y_min});
        }
    }
    
    // Check intersection with top edge (y = y_max)
    if (std::abs(m) > epsilon) {
        double x_top = (y_max - q) / m;
        if (x_top >= x_min - epsilon && x_top <= x_max + epsilon) {
            intersections.push_back({x_top, y_max});
        }
    }
    
    // Remove duplicates
    std::vector<std::pair<double, double>> unique_intersections;
    for (const auto& point : intersections) {
        bool is_duplicate = false;
        for (const auto& existing : unique_intersections) {
            if (std::abs(point.first - existing.first) < epsilon && 
                std::abs(point.second - existing.second) < epsilon) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            unique_intersections.push_back(point);
        }
    }
    
    return unique_intersections;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_collision_generator");
    ros::NodeHandle nh;
    
    // Use a single-threaded spinner as the node performs a single, one-off task.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // MoveIt planning scene interface to add collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Service client to get line coefficients
    ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    tiago_iaslab_simulation::Coeffs coeffs_srv;
    
    // TF listener for coordinate frame transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(2.0).sleep();
    
    // Wait for the service to be available
    ROS_INFO("Waiting for /straight_line_srv service...");
    coeffs_client.waitForExistence();
    
    // Call the service to get a new line
    float m = 0.0, q = 0.0;
    coeffs_srv.request.ready = true;
    if (!coeffs_client.call(coeffs_srv))
    {
        ROS_ERROR("Failed to call /straight_line_srv. Shutting down.");
        ros::shutdown();
        return 1;
    }
    
    m = coeffs_srv.response.coeffs[0];
    q = coeffs_srv.response.coeffs[1];

    // Define the table's pose manually using the provided offset from the tag frame.
    // The previous code failed because it couldn't retrieve the table from the planning scene.
    geometry_msgs::PoseStamped table_pose_in_tag_frame;
    table_pose_in_tag_frame.header.frame_id = TAG_FRAME;
    table_pose_in_tag_frame.header.stamp = ros::Time(0);
    table_pose_in_tag_frame.pose.position.x = -1.5;
    table_pose_in_tag_frame.pose.position.y = 1.5;
    table_pose_in_tag_frame.pose.position.z = -TABLE_HEIGHT / 2.0; // Assume tag is on top of the table
    table_pose_in_tag_frame.pose.orientation.w = 1.0; // No rotation for simplicity
    
    ROS_INFO("Using hardcoded table pose in TAG_FRAME: x=%.4f, y=%.4f", 
             table_pose_in_tag_frame.pose.position.x,
             table_pose_in_tag_frame.pose.position.y);


    double table_x_min = table_pose_in_tag_frame.pose.position.x - TABLE_WIDTH / 2.0;
    double table_x_max = table_pose_in_tag_frame.pose.position.x + TABLE_WIDTH / 2.0;
    double table_y_min = table_pose_in_tag_frame.pose.position.y - TABLE_DEPTH / 2.0;
    double table_y_max = table_pose_in_tag_frame.pose.position.y + TABLE_DEPTH / 2.0;
    
    // Find where the line intersects the table. The result is a vector of 2D points.
    auto intersections = findLineTableIntersections(m, q, table_x_min, table_x_max, table_y_min, table_y_max);
    
    if (intersections.size() < 2) {
        ROS_ERROR("Line (y=%.3fx+%.3f) does not properly intersect the table. Shutting down.", m, q);
        ros::shutdown();
        return 1;
    }
    
    // Remove all previous placement collision objects before adding new ones
    planning_scene_interface.removeCollisionObjects({"placement_marker_0", "placement_marker_1", "placement_marker_2"});

    // Sort intersections by x-coordinate to get start and end points of the line segment on the table
    std::sort(intersections.begin(), intersections.end());
    double line_start_x = intersections[0].first;
    double line_start_y = intersections[0].second;
    double line_end_x = intersections.back().first;
    double line_end_y = intersections.back().second;
    
    // Define placement points along the line segment
    int num_objects = 3;
    double margin_ratio = 0.1; // 10% margin from each end
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    // Create placement position collision objects
    for (int i = 0; i < num_objects; ++i)
    {
        // Calculate position along the line *segment* that is on the table
        double t = margin_ratio + (1.0 - 2.0 * margin_ratio) * i / (num_objects - 1);
        double x = line_start_x + t * (line_end_x - line_start_x);
        double y = line_start_y + t * (line_end_y - line_start_y);
        
        // Set pose in tag frame, then transform to base frame
        geometry_msgs::PoseStamped place_pose_tag;
        place_pose_tag.header.frame_id = TAG_FRAME;
        place_pose_tag.header.stamp = ros::Time(0);
        place_pose_tag.pose.position.x = x;
        place_pose_tag.pose.position.y = y;
        place_pose_tag.pose.position.z = 0.0; // Z is 0 in the tag frame as the tag is on the table surface
        
        tf2::Quaternion q_down;
        q_down.setRPY(M_PI, 0, 0);
        place_pose_tag.pose.orientation = tf2::toMsg(q_down);
        
        // Transform to base frame for collision objects
        geometry_msgs::PoseStamped place_pose_base;
        try
        {
            tfBuffer.transform(place_pose_tag, place_pose_base, BASE_FRAME, ros::Duration(1.0));
            
            // Create collision object at placement position
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = BASE_FRAME;
            collision_object.id = "placement_marker_" + std::to_string(i);
            
            // Create a small box to mark the position
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.05;
            primitive.dimensions[1] = 0.05;
            primitive.dimensions[2] = 0.02;
            
            geometry_msgs::Pose box_pose = place_pose_base.pose;
            // Set the Z position to the requested height, regardless of the table's height
            box_pose.position.z = PLACEMENT_Z_HEIGHT; 
            
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("TF transform error for position %d: %s", i, ex.what());
        }
    }
    
    // Add all new placement collision objects to planning scene
    planning_scene_interface.addCollisionObjects(collision_objects);
    
    // Add a small delay to ensure the objects are added before the node shuts down
    ros::Duration(0.1).sleep();

    ROS_INFO("Successfully added 3 placement markers.");
    ros::shutdown();
    
    return 0;
}
