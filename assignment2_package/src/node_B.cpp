/**
 * @file node_B.cpp
 * @brief AprilTag detection and object pose management node
 * 
 * This node handles:
 * - Real-time AprilTag detection from camera feed
 * - Pose transformation from camera frame to robot base frame
 * - Object pose publishing for other nodes to consume
 * - Priority-based object selection service with bin management
 * - Tracking of picked objects to avoid duplicates
 * 
 * Object Organization:
 * - Bin 1: Cylinders (Tag IDs 1-3)
 * - Bin 2: Cubes (Tag IDs 4-6) 
 * - Bin 3: Triangular Prisms (Tag IDs 7-9)
 * - Tag 10: Static reference tag (on placing table)
 * 
 * Priority System:
 * - Objects are selected by bin priority: Bin 1 -> Bin 2 -> Bin 3
 * - Once any object from a bin is picked, the entire bin is marked as unavailable
 * - This ensures systematic processing of different object types
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <unordered_set>
#include "assignment2_package/GetObjectPose.h"
#include "assignment2_package/ObjectPoseArray.h"
#include "assignment2_package/ObjectPose.h"

// =============================================================================
// CONSTANTS AND CONFIGURATION
// =============================================================================

// Frame definitions for coordinate transformations
static const std::string CAMERA_FRAME = "xtion_rgb_optical_frame"; // Camera optical frame
static const std::string BASE_FRAME = "base_footprint";            // Robot base frame for planning
static const int REF_TAG_ID = 10;                                  // Reference tag on placing table (not for manipulation)

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// ROS communication interfaces
ros::Subscriber detections_sub;  // Subscriber for AprilTag detections
ros::ServiceServer get_obj_pose_srv; // Service to provide object poses with priority
ros::Publisher object_pose_pub;   // Publisher for all detected object poses

// TF system for coordinate transformations
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListenerPtr;

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief Structure to store detected object information
 * Contains object ID and its pose transformed to the robot base frame
 */
struct DetectedObject
{
    int id;                          // AprilTag ID (1-9 for objects, 10 for reference)
    geometry_msgs::PoseStamped pose; // Object pose in base_footprint frame
};

// Object tracking and state management
std::vector<DetectedObject> current_objects; // Currently detected objects (updated each detection cycle)
std::unordered_set<int> picked_ids;          // Individual object IDs that have been picked
std::unordered_set<int> picked_bins;         // Bin IDs that have been marked as picked

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * @brief Map tag ID to bin ID for priority-based selection
 * 
 * Objects are organized into bins by type:
 * - Bin 1: Cylinders (Tags 1-3)
 * - Bin 2: Cubes (Tags 4-6)
 * - Bin 3: Triangular Prisms (Tags 7-9)
 * 
 * @param tag_id The AprilTag ID (1-9)
 * @return Bin ID (1-3) or 0 if invalid
 */
int getBinId(int tag_id)
{
    if (tag_id >= 1 && tag_id <= 3)
    {
        return 1; // Cylinder bin
    }
    else if (tag_id >= 4 && tag_id <= 6)
    {
        return 2; // Cube bin
    }
    else if (tag_id >= 7 && tag_id <= 9)
    {
        return 3; // Triangular prism bin
    }
    return 0; // Invalid bin (reference tag or out of range)
}

// =============================================================================
// CALLBACK FUNCTIONS
// =============================================================================

/**
 * @brief Callback for AprilTag detection messages
 * 
 * Process flow:
 * 1. Clear previous detection results
 * 2. Obtain camera-to-base transform for coordinate conversion
 * 3. Transform each detected tag pose from camera frame to base frame
 * 4. Filter out already picked objects
 * 5. Publish updated object pose array for other nodes
 * 
 * @param msg AprilTag detection array from apriltag_ros
 */
void detectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // Clear previous detection cycle results
    current_objects.clear();

    // Obtain coordinate transformation from camera to robot base
    geometry_msgs::TransformStamped cam_to_base;
    try
    {
        cam_to_base = tfBuffer.lookupTransform(BASE_FRAME, msg->header.frame_id,
                                               msg->header.stamp, ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(1.0, "Node B: TF lookup failed for camera to base: %s", ex.what());
        return;
    }

    // Process each detected AprilTag
    for (const auto &detection : msg->detections)
    {
        // Skip detections without valid ID
        if (detection.id.empty())
            continue;

        int tag_id = detection.id[0];

        // Extract pose from detection message (in camera frame)
        geometry_msgs::PoseStamped tag_pose_cam;
        tag_pose_cam.header = detection.pose.header;
        tag_pose_cam.pose = detection.pose.pose.pose;

        // Transform pose from camera frame to robot base frame
        geometry_msgs::PoseStamped tag_pose_base;
        tf2::doTransform(tag_pose_cam, tag_pose_base, cam_to_base);
        tag_pose_base.header.frame_id = BASE_FRAME;

        // Only store objects that haven't been picked yet
        if (picked_ids.find(tag_id) == picked_ids.end())
        {
            DetectedObject obj;
            obj.id = tag_id;
            obj.pose = tag_pose_base;
            current_objects.push_back(obj);

            // Throttled logging to avoid spam (once per 5 seconds per tag)
            ROS_INFO_THROTTLE(5.0, "Node B: Detected object ID %d at [%.2f, %.2f, %.2f]",
                              tag_id,
                              tag_pose_base.pose.position.x,
                              tag_pose_base.pose.position.y,
                              tag_pose_base.pose.position.z);
        }
    }

    // Publish all currently detected (non-picked) objects for consumption by other nodes
    assignment2_package::ObjectPoseArray msg_out;
    for (const auto &obj : current_objects)
    {
        assignment2_package::ObjectPose entry;
        entry.id = obj.id;
        entry.pose = obj.pose;
        msg_out.objects.push_back(entry);
    }
    object_pose_pub.publish(msg_out);
}

// =============================================================================
// SERVICE CALLBACKS
// =============================================================================

/**
 * @brief Service to provide object poses with priority-based selection
 * 
 * Selection Strategy:
 * 1. Process bins in priority order: Cylinders (1) -> Cubes (2) -> Prisms (3)
 * 2. Skip bins that have already been picked from
 * 3. Return first available object from highest priority bin
 * 4. Mark the object's entire bin as picked to ensure systematic processing
 * 
 * This approach ensures:
 * - Consistent object type processing order
 * - No mixing of object types during manipulation sequences
 * - Simplified bin management for the pick-and-place workflow
 * 
 * @param req Service request (no input parameters needed)
 * @param res Service response containing object ID and pose
 * @return true if object provided successfully, false if no suitable objects available
 */
bool getObjectPoseService(assignment2_package::GetObjectPose::Request &req,
                          assignment2_package::GetObjectPose::Response &res)
{
    // Check if any objects are currently detected
    if (current_objects.empty())
    {
        ROS_WARN("Node B: No objects currently detected. Cannot provide object pose.");
        return false;
    }

    // Define bin processing priority (cylinders first, then cubes, then prisms)
    std::vector<int> bin_priority = {1, 2, 3};

    // Iterate through bins in priority order
    for (int priority_bin : bin_priority)
    {
        // Skip bins that have already been processed
        if (picked_bins.find(priority_bin) != picked_bins.end())
        {
            ROS_DEBUG("Node B: Bin %d already picked, skipping", priority_bin);
            continue;
        }

        // Search for objects in the current priority bin
        for (const auto &obj : current_objects)
        {
            int bin_id = getBinId(obj.id);

            if (bin_id == priority_bin)
            {
                // Found an available object in the current priority bin
                res.obj_id = obj.id;
                res.obj_pose = obj.pose;

                // Mark both the specific object and its entire bin as picked
                // This prevents mixing object types and ensures systematic processing
                picked_ids.insert(obj.id);
                picked_bins.insert(bin_id);

                ROS_INFO("Node B: Providing object ID %u from priority bin %u. Bin is now marked as picked.",
                         res.obj_id, bin_id);
                return true;
            }
        }

        ROS_DEBUG("Node B: No objects found in priority bin %d", priority_bin);
    }

    // No suitable objects found in any available bin
    ROS_WARN("Node B: No suitable objects found to provide. All available bins may have been picked.");
    return false;
}

// =============================================================================
// MAIN FUNCTION
// =============================================================================

/**
 * @brief Main function - initializes the node and starts processing
 * 
 * Setup process:
 * 1. Initialize ROS node and communication interfaces
 * 2. Set up TF listener for coordinate transformations
 * 3. Subscribe to AprilTag detection topic
 * 4. Advertise object pose service and topic
 * 5. Enter main processing loop
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_B");
    ros::NodeHandle nh;

    // Initialize TF system for coordinate transformations
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);

    // Set up ROS communication interfaces
    detections_sub = nh.subscribe("tag_detections", 1, detectionsCallback);        // Input: AprilTag detections
    get_obj_pose_srv = nh.advertiseService("get_object_pose", getObjectPoseService); // Service: Priority-based object selection
    object_pose_pub = nh.advertise<assignment2_package::ObjectPoseArray>("object_poses", 1); // Output: All detected poses

    ROS_INFO("Node B: Detection node started, waiting for tag detections...");

    // Enter main processing loop
    ros::spin();

    // Cleanup
    delete tfListenerPtr;
    return 0;
}