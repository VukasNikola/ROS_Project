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
std::unordered_set<int> picked_ids; // set of IDs already picked

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

        // Skip reference tag (ID 10) - we don't pick this one
        if (tag_id == REF_TAG_ID)
        {
            ROS_INFO_THROTTLE(0.5, "Node B: Reference tag (ID 10) detected.");
            continue;
        }

        // Only process object tags (1-9)
        if (tag_id < 1 || tag_id > 9)
            continue;

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

            ROS_INFO("Node B: Detected object ID %d at [%.2f, %.2f, %.2f]",
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

// Service to remove id from detected objects
bool getObjectPoseService(assignment2_package::GetObjectPose::Request &req,
                          assignment2_package::GetObjectPose::Response &res)
{
    if (current_objects.empty())
    {
        ROS_WARN("Node B: No objects currently detected. Cannot provide object pose.");
        return false;
    }

    // Take the first available object
    DetectedObject obj = current_objects.front();

    // Mark it as picked
    picked_ids.insert(obj.id);

    // Return the pose in base frame
    res.obj_id = obj.id;
    res.obj_pose = obj.pose;

    ROS_INFO("Node B: Providing object ID %u pose to requester.", res.obj_id);
    return true;
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

// This node continuously listens for AprilTag detections,
// transforms each unpicked tag’s pose into the robot’s base frame,
// and makes those fresh poses available on demand. On every incoming tag_detections message it clears its previous list,
// looks up the latest camera-to-base transform,
// and for each detected tag (IDs 1–9, excluding the special reference tag 10) that hasn’t yet been “picked,”
// computes its pose in the base frame and adds it to current_objects.
// A ROS service (get_object_pose) then lets another node request the next available object: once it hands out a tag’s pose,
// it marks that tag ID in picked_ids so it won’t be offered again.
// Thus, at any moment, current_objects holds the up-to-date base-frame poses of all remaining, unpicked tags.