#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <unordered_set>
#include "assignment2_package/GetObjectPose.h"
#include "assignment2_package/ObjectPoseArray.h"
#include "assignment2_package/ObjectPose.h"

// Node B: Detect AprilTags and compute object poses
static const std::string CAMERA_FRAME = "xtion_rgb_optical_frame";
static const std::string BASE_FRAME = "base_footprint"; // Robot base frame
static const int REF_TAG_ID = 10;                       // AprilTag ID used as static reference (on placing table)

ros::Subscriber detections_sub;
ros::ServiceServer get_obj_pose_srv;
ros::Publisher object_pose_pub;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListenerPtr;

// Data structure to store latest detected poses
struct DetectedObject
{
    int id;
    geometry_msgs::PoseStamped pose; // pose in base frame
};

std::vector<DetectedObject> current_objects;
std::unordered_set<int> picked_ids;  // set of IDs already picked
std::unordered_set<int> picked_bins; // new set to store picked bin IDs

// Helper function to get the bin ID for a given tag ID
int getBinId(int tag_id)
{
    if (tag_id >= 1 && tag_id <= 3)
    {
        return 1;
    }
    else if (tag_id >= 4 && tag_id <= 6)
    {
        return 2;
    }
    else if (tag_id >= 7 && tag_id <= 9)
    {
        return 3;
    }
    return 0; // Invalid bin
}

// Callback for AprilTag detections
void detectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // Clear previous detections
    current_objects.clear();

    // Find transform from camera to base
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

    // Process each detected tag
    for (const auto &detection : msg->detections)
    {
        if (detection.id.empty())
            continue;

        int tag_id = detection.id[0];

        // Get pose of tag in camera frame
        geometry_msgs::PoseStamped tag_pose_cam;
        tag_pose_cam.header = detection.pose.header;
        tag_pose_cam.pose = detection.pose.pose.pose;

        // Transform to base frame
        geometry_msgs::PoseStamped tag_pose_base;
        tf2::doTransform(tag_pose_cam, tag_pose_base, cam_to_base);
        tag_pose_base.header.frame_id = BASE_FRAME;

        // Store if not already picked
        if (picked_ids.find(tag_id) == picked_ids.end())
        {
            DetectedObject obj;
            obj.id = tag_id;
            obj.pose = tag_pose_base;
            current_objects.push_back(obj);

            // Only print detection info once every 5 seconds per tag ID
            ROS_INFO_THROTTLE(5.0, "Node B: Detected object ID %d at [%.2f, %.2f, %.2f]",
                              tag_id,
                              tag_pose_base.pose.position.x,
                              tag_pose_base.pose.position.y,
                              tag_pose_base.pose.position.z);
        }
    }
    // Publish all current objects
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

// Service to get an object pose and remove it and its group from the detected objects
bool getObjectPoseService(assignment2_package::GetObjectPose::Request &req,
                          assignment2_package::GetObjectPose::Response &res)
{
    if (current_objects.empty())
    {
        ROS_WARN("Node B: No objects currently detected. Cannot provide object pose.");
        return false;
    }

    // Define bin priority order 
    std::vector<int> bin_priority = {1, 2, 3};

    // Try bins in priority order
    for (int priority_bin : bin_priority)
    {
        // Skip if this bin has already been picked
        if (picked_bins.find(priority_bin) != picked_bins.end())
        {
            ROS_DEBUG("Node B: Bin %d already picked, skipping", priority_bin);
            continue;
        }

        // Look for objects in this priority bin
        for (const auto &obj : current_objects)
        {
            int bin_id = getBinId(obj.id);

            if (bin_id == priority_bin)
            {
                // Found an object in the current priority bin
                res.obj_id = obj.id;
                res.obj_pose = obj.pose;

                // Mark the object and its entire bin as picked
                picked_ids.insert(obj.id);
                picked_bins.insert(bin_id);

                ROS_INFO("Node B: Providing object ID %u from priority bin %u. Bin is now marked as picked.",
                         res.obj_id, bin_id);
                return true;
            }
        }

        ROS_DEBUG("Node B: No objects found in priority bin %d", priority_bin);
    }

    // If we get here, no suitable objects were found
    ROS_WARN("Node B: No suitable objects found to provide. All available bins may have been picked.");
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_B");
    ros::NodeHandle nh;

    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);

    // Subscribe to AprilTag detections
    detections_sub = nh.subscribe("tag_detections", 1, detectionsCallback);

    // Advertise services
    get_obj_pose_srv = nh.advertiseService("get_object_pose", getObjectPoseService);
    object_pose_pub = nh.advertise<assignment2_package::ObjectPoseArray>("object_poses", 1);

    ROS_INFO("Node B: Detection node started, waiting for tag detections...");

    ros::spin();

    delete tfListenerPtr;
    return 0;
}