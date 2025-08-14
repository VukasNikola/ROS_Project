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

static const std::string TAG_FRAME = "tag_10";
static const std::string BASE_FRAME = "base_footprint";
static const double TABLE_WIDTH = 1.07;
static const double TABLE_DEPTH = 1.07;
static const double TABLE_HEIGHT = 0.77;

// Offset to the point of the table where there is a good spot to place objects
static const double TABLE_OFFSET_X = 0.36;
static const double TABLE_OFFSET_Y = 0.18;

const double EPSILON = 1e-6;

struct LineInfo {
    double m, q;
    double closest_point_x, closest_point_y;
    double distance_to_table_center;
    std::vector<std::pair<double, double>> placement_points;
    bool all_points_valid;
};

// Structure to store the selected line and placement data
struct StoredPlacementData {
    bool is_valid;
    double line_m, line_q;
    double table_center_x, table_center_y;
    std::vector<std::pair<double, double>> placement_points_tag_frame;
    
    StoredPlacementData() : is_valid(false) {}
};

// Global storage for the selected placement data
StoredPlacementData g_stored_data;

/**
 * @brief Checks if a point is within a rectangle.
 */
bool is_point_on_table(double x, double y, double x_min, double x_max, double y_min, double y_max)
{
    return (x >= x_min - EPSILON && x <= x_max + EPSILON &&
            y >= y_min - EPSILON && y <= y_max + EPSILON);
}

/**
 * @brief Calculate the closest point on line to table center and evaluate line
 */
LineInfo calculateLineInfo(double m, double q, double table_center_x, double table_center_y,
                          double table_x_min, double table_x_max, double table_y_min, double table_y_max)
{
    LineInfo info;
    info.m = m;
    info.q = q;
    
    // Find the closest point on the line to the table center
    if (std::abs(m) < EPSILON) {
        // Horizontal line: y = q
        info.closest_point_x = table_center_x;
        info.closest_point_y = q;
    } else if (std::abs(1.0/m) < EPSILON) {
        // Vertical line: x = -q/m
        info.closest_point_x = -q/m;
        info.closest_point_y = table_center_y;
    } else {
        // General case: find perpendicular intersection
        double m_perp = -1.0 / m;
        double q_perp = table_center_y - m_perp * table_center_x;
        info.closest_point_x = (q_perp - q) / (m - m_perp);
        info.closest_point_y = m * info.closest_point_x + q;
    }
    
    // Calculate distance from closest point to table center (this is the line's distance to center)
    info.distance_to_table_center = std::sqrt(
        std::pow(info.closest_point_x - table_center_x, 2) + 
        std::pow(info.closest_point_y - table_center_y, 2)
    );
    
    // Calculate direction vector along the line
    double dir_x, dir_y;
    if (std::abs(m) < EPSILON) {
        // Horizontal line
        dir_x = 1.0;
        dir_y = 0.0;
    } else if (std::abs(1.0/m) < EPSILON) {
        // Vertical line
        dir_x = 0.0;
        dir_y = 1.0;
    } else {
        // General case
        dir_x = 1.0;
        dir_y = m;
        double length = std::sqrt(dir_x * dir_x + dir_y * dir_y);
        dir_x /= length;
        dir_y /= length;
    }
    
    // Generate placement points: center point ± 0.15m along the line
    double spacing = 0.15;
    std::vector<std::pair<double, double>> candidate_points;
    
    // Center point (closest to table center)
    candidate_points.push_back({info.closest_point_x, info.closest_point_y});
    
    // Point 0.15m "above" (in positive direction along line)
    candidate_points.push_back({
        info.closest_point_x + spacing * dir_x,
        info.closest_point_y + spacing * dir_y
    });
    
    // Point 0.15m "below" (in negative direction along line)
    candidate_points.push_back({
        info.closest_point_x - spacing * dir_x,
        info.closest_point_y - spacing * dir_y
    });
    
    // Check if all points are on the table
    info.all_points_valid = true;
    for (const auto& point : candidate_points) {
        if (is_point_on_table(point.first, point.second, table_x_min, table_x_max, table_y_min, table_y_max)) {
            info.placement_points.push_back(point);
        } else {
            info.all_points_valid = false;
            ROS_WARN("Point (%.3f, %.3f) is off the table", point.first, point.second);
        }
    }
    
    // Only consider valid if all 3 points fit on table
    if (info.placement_points.size() != 3) {
        info.all_points_valid = false;
        info.placement_points.clear();
    }
    
    return info;
}

/**
 * @brief Store the selected line and placement points for future use
 */
void storePlacementData(const LineInfo& best_line, double table_center_x, double table_center_y)
{
    g_stored_data.is_valid = true;
    g_stored_data.line_m = best_line.m;
    g_stored_data.line_q = best_line.q;
    g_stored_data.table_center_x = table_center_x;
    g_stored_data.table_center_y = table_center_y;
    g_stored_data.placement_points_tag_frame = best_line.placement_points;
    
    ROS_INFO("=== PLACEMENT DATA STORED ===");
    ROS_INFO("Line: y = %.3fx + %.3f", g_stored_data.line_m, g_stored_data.line_q);
    ROS_INFO("Table center in tag_10: (%.3f, %.3f)", g_stored_data.table_center_x, g_stored_data.table_center_y);
    ROS_INFO("Stored %zu placement points in tag_10 frame", g_stored_data.placement_points_tag_frame.size());
}

/**
 * @brief Apply the stored placement markers to the planning scene
 * This function can be called anytime when tag_10 is visible
 */
bool applyStoredPlacements(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                          tf2_ros::Buffer& tfBuffer)
{
    if (!g_stored_data.is_valid) {
        ROS_ERROR("No stored placement data available. Run line selection first.");
        return false;
    }
    
    ROS_INFO("=== APPLYING STORED PLACEMENTS ===");
    
    // Remove any existing markers
    planning_scene_interface.removeCollisionObjects({
        "placement_marker_0", "placement_marker_1", "placement_marker_2", "table_center_marker"
    });
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    // Add table center marker
    geometry_msgs::PoseStamped table_center_pose_tag;
    table_center_pose_tag.header.frame_id = TAG_FRAME;
    table_center_pose_tag.header.stamp = ros::Time(0);
    table_center_pose_tag.pose.position.x = g_stored_data.table_center_x;
    table_center_pose_tag.pose.position.y = g_stored_data.table_center_y;
    table_center_pose_tag.pose.position.z = 0.0;
    
    tf2::Quaternion q_center;
    q_center.setRPY(M_PI, 0, 0);
    table_center_pose_tag.pose.orientation = tf2::toMsg(q_center);
    
    geometry_msgs::PoseStamped table_center_pose_base;
    try {
        tfBuffer.transform(table_center_pose_tag, table_center_pose_base, BASE_FRAME, ros::Duration(1.0));
        
        moveit_msgs::CollisionObject table_center_object;
        table_center_object.header.frame_id = BASE_FRAME;
        table_center_object.id = "table_center_marker";
        
        shape_msgs::SolidPrimitive center_primitive;
        center_primitive.type = center_primitive.CYLINDER;
        center_primitive.dimensions.resize(2);
        center_primitive.dimensions[0] = 0.03;  // height
        center_primitive.dimensions[1] = 0.03;  // radius
        
        geometry_msgs::Pose center_pose = table_center_pose_base.pose;
        center_pose.position.z = TABLE_HEIGHT + 0.015;
        
        table_center_object.primitives.push_back(center_primitive);
        table_center_object.primitive_poses.push_back(center_pose);
        table_center_object.operation = table_center_object.ADD;
        collision_objects.push_back(table_center_object);
        
        ROS_INFO("TABLE CENTER: tag_10(%.3f, %.3f) -> base(%.3f, %.3f, %.3f)",
                 g_stored_data.table_center_x, g_stored_data.table_center_y,
                 center_pose.position.x, center_pose.position.y, center_pose.position.z);
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("TF transform error for table center: %s", ex.what());
        return false;
    }
    
    // Add placement markers using stored positions
    for (int i = 0; i < g_stored_data.placement_points_tag_frame.size(); ++i) {
        double x = g_stored_data.placement_points_tag_frame[i].first;
        double y = g_stored_data.placement_points_tag_frame[i].second;
        
        geometry_msgs::PoseStamped place_pose_tag;
        place_pose_tag.header.frame_id = TAG_FRAME;
        place_pose_tag.header.stamp = ros::Time(0);
        place_pose_tag.pose.position.x = x;
        place_pose_tag.pose.position.y = y;
        place_pose_tag.pose.position.z = 0.0;
        
        tf2::Quaternion q_down;
        q_down.setRPY(M_PI, 0, 0);
        place_pose_tag.pose.orientation = tf2::toMsg(q_down);
        
        geometry_msgs::PoseStamped place_pose_base;
        try {
            tfBuffer.transform(place_pose_tag, place_pose_base, BASE_FRAME, ros::Duration(1.0));
            
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = BASE_FRAME;
            collision_object.id = "placement_marker_" + std::to_string(i);
            
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.05;
            primitive.dimensions[1] = 0.05;
            primitive.dimensions[2] = 0.02;
            
            geometry_msgs::Pose box_pose = place_pose_base.pose;
            box_pose.position.z = TABLE_HEIGHT;
            
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);
            
            std::string position_name = (i == 0) ? "CENTER" : (i == 1 ? "ABOVE" : "BELOW");
            ROS_INFO("Marker %d (%s): tag_10(%.3f, %.3f) -> base(%.3f, %.3f, %.3f)",
                     i, position_name.c_str(), x, y, 
                     box_pose.position.x, box_pose.position.y, box_pose.position.z);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("TF transform error for position %d: %s", i, ex.what());
            return false;
        }
    }
    
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(0.1).sleep();
    
    ROS_INFO("Successfully applied stored placements! (%zu markers placed)", 
             g_stored_data.placement_points_tag_frame.size());
    return true;
}

/**
 * @brief Get the current pose of a specific placement marker in base_footprint frame
 */
bool getPlacementPose(int marker_index, geometry_msgs::PoseStamped& pose_out, tf2_ros::Buffer& tfBuffer)
{
    if (!g_stored_data.is_valid) {
        ROS_ERROR("No stored placement data available");
        return false;
    }
    
    if (marker_index < 0 || marker_index >= g_stored_data.placement_points_tag_frame.size()) {
        ROS_ERROR("Invalid marker index %d. Valid range: 0-%zu", 
                  marker_index, g_stored_data.placement_points_tag_frame.size()-1);
        return false;
    }
    
    double x = g_stored_data.placement_points_tag_frame[marker_index].first;
    double y = g_stored_data.placement_points_tag_frame[marker_index].second;
    
    geometry_msgs::PoseStamped pose_tag;
    pose_tag.header.frame_id = TAG_FRAME;
    pose_tag.header.stamp = ros::Time(0);
    pose_tag.pose.position.x = x;
    pose_tag.pose.position.y = y;
    pose_tag.pose.position.z = 0.0;
    
    tf2::Quaternion q_down;
    q_down.setRPY(M_PI, 0, 0);
    pose_tag.pose.orientation = tf2::toMsg(q_down);
    
    try {
        tfBuffer.transform(pose_tag, pose_out, BASE_FRAME, ros::Duration(1.0));
        pose_out.pose.position.z = TABLE_HEIGHT;  // Adjust to table surface
        return true;
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("TF transform error for marker %d: %s", marker_index, ex.what());
        return false;
    }
}

/**
 * @brief Check if stored placement data is available
 */
bool hasStoredPlacements()
{
    return g_stored_data.is_valid;
}

/**
 * @brief Clear stored placement data
 */
void clearStoredPlacements()
{
    g_stored_data.is_valid = false;
    ROS_INFO("Cleared stored placement data");
}

/**
 * @brief Find and store the best line (run this once when tag_10 is first visible)
 */
bool findAndStoreBestLine(ros::ServiceClient& coeffs_client)
{
    ROS_INFO("=== FINDING AND STORING BEST LINE ===");
    
    tiago_iaslab_simulation::Coeffs coeffs_srv;
    
    double table_center_x = TABLE_OFFSET_X;
    double table_center_y = TABLE_OFFSET_Y;
    double table_x_min = table_center_x - TABLE_WIDTH / 2.0;
    double table_x_max = table_center_x + TABLE_WIDTH / 2.0;
    double table_y_min = table_center_y - TABLE_DEPTH / 2.0;
    double table_y_max = table_center_y + TABLE_DEPTH / 2.0;
    
    std::vector<LineInfo> valid_lines;
    
    // Iterate through lines to find the best one
    for (int iteration = 0; iteration < 15; ++iteration) {
        coeffs_srv.request.ready = true;
        if (!coeffs_client.call(coeffs_srv)) {
            ROS_ERROR("Failed to call /straight_line_srv on iteration %d", iteration);
            continue;
        }
        
        double m = coeffs_srv.response.coeffs[0];
        double q = coeffs_srv.response.coeffs[1];
        
        ROS_INFO("Iteration %d: Line y = %.3fx + %.3f", iteration, m, q);
        
        LineInfo line_info = calculateLineInfo(m, q, table_center_x, table_center_y,
                                             table_x_min, table_x_max, table_y_min, table_y_max);
        
        ROS_INFO("  - Distance to table center: %.4f", line_info.distance_to_table_center);
        
        if (line_info.all_points_valid) {
            valid_lines.push_back(line_info);
            ROS_INFO("  - All 3 placement points are valid");
        } else {
            ROS_WARN("  - Some placement points are off the table");
        }
    }
    
    if (valid_lines.empty()) {
        ROS_ERROR("No valid lines found after 15 iterations");
        return false;
    }
    
    // Find the best line
    auto best_line = std::min_element(valid_lines.begin(), valid_lines.end(),
        [](const LineInfo& a, const LineInfo& b) {
            return a.distance_to_table_center < b.distance_to_table_center;
        });
    
    // Store the best line data
    storePlacementData(*best_line, table_center_x, table_center_y);
    
    return true;
}

// Timer callback function to update placements every second
void updatePlacementsCallback(const ros::TimerEvent&, 
                             moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                             tf2_ros::Buffer& tfBuffer)
{
    if (!g_stored_data.is_valid) {
        return;  // No stored data, skip update
    }
    
    // Try to update placements (will fail silently if tag_10 not visible)
    if (!applyStoredPlacements(planning_scene_interface, tfBuffer)) {
        // Reduce log spam - only show warning occasionally
        static int failed_count = 0;
        failed_count++;
        if (failed_count % 10 == 1) {  // Show warning every 10 failures
            ROS_WARN("Could not update placement markers (tag_10 not visible?) - attempt %d", failed_count);
        }
    } else {
        ROS_DEBUG("Placement markers updated successfully");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_collision_generator");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    ros::ServiceClient coeffs_client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    
    ros::Duration(2.0).sleep();
    
    ROS_INFO("Waiting for /straight_line_srv service...");
    coeffs_client.waitForExistence();
    
    // Step 1: Find and store the best line
    if (!findAndStoreBestLine(coeffs_client)) {
        ROS_ERROR("Failed to find a valid line. Exiting.");
        ros::shutdown();
        return 1;
    }
    
    // Step 2: Apply the stored placements initially
    if (!applyStoredPlacements(planning_scene_interface, tfBuffer)) {
        ROS_ERROR("Failed to apply stored placements initially. Exiting.");
        ros::shutdown();
        return 1;
    }
    
    // Step 3: Set up timer to update placements every second
    ros::Timer update_timer = nh.createTimer(
        ros::Duration(1.0),  // 1 second interval
        boost::bind(&updatePlacementsCallback, _1, boost::ref(planning_scene_interface), boost::ref(tfBuffer))
    );
    
    ROS_INFO("=== READY FOR USE ===");
    ROS_INFO("Placement markers will be updated every second when tag_10 is visible");
    ROS_INFO("Use getPlacementPose(index, pose, tfBuffer) to get specific marker poses");
    
    // Example: Get pose of center marker (index 0)
    geometry_msgs::PoseStamped center_pose;
    if (getPlacementPose(0, center_pose, tfBuffer)) {
        ROS_INFO("CENTER marker pose in base_footprint: (%.3f, %.3f, %.3f)",
                 center_pose.pose.position.x, center_pose.pose.position.y, center_pose.pose.position.z);
    }
    
    // Keep the node running with timer updates
    ROS_INFO("Node running... Press Ctrl+C to exit");
    ros::waitForShutdown();
    
    return 0;
}