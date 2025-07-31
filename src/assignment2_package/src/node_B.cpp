#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <unordered_set>
#include "assignment2_package/GetObjectPose.h" 

// Node B: Detect AprilTags and compute object poses
static const std::string CAMERA_FRAME = "xtion_rgb_optical_frame";  // example camera frame (adjust based on simulation)
static const std::string BASE_FRAME = "base_footprint";            // robot base frame
static const int REF_TAG_ID = 10;  // AprilTag ID used as static reference (on placing table)

ros::Subscriber detections_sub;
ros::ServiceServer get_obj_pose_srv;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListenerPtr;

// Data structure to store latest detected poses (in base frame or in reference frame as needed)
struct DetectedObject {
    int id;
    geometry_msgs::PoseStamped pose;  // pose in base frame
};
std::vector<DetectedObject> current_objects;
std::unordered_set<int> picked_ids;  // set of IDs already picked

// Transform from camera to reference tag (if reference tag seen)
geometry_msgs::TransformStamped refCamTransform; 
bool haveRefTransform = false;

// Callback for AprilTag detections
void detectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    // Clear previous detections
    current_objects.clear();
    // Find transform from camera to base (static from robot URDF)
    geometry_msgs::TransformStamped cam_to_base;
    try {
        cam_to_base = tfBuffer.lookupTransform(BASE_FRAME, msg->header.frame_id,  // camera frame to base
                                               msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Node B: TF lookup failed for camera to base: %s", ex.what());
        // If we can't get camera->base, return (we'll try again next callback)
        return;
    }
    // Check each detected tag
    for (auto& detection : msg->detections) {
        if (detection.id.empty()) continue;
        int tag_id = detection.id[0];
        // Get the pose of the tag relative to camera (PoseWithCovarianceStamped in detection)
        geometry_msgs::PoseStamped tag_pose_cam = detection.pose.pose;  // Pose of tag in camera frame
        tag_pose_cam.header = detection.pose.header;  // ensure frame_id and stamp are set
        // If this is the reference tag (ID 10), record the transform camera->tag10
        if (tag_id == REF_TAG_ID) {
            try {
                // We can derive transform from camera to tag reference directly from the pose
                // Alternatively, use tfBuffer.transform if apriltag publishes tf frames (if publish_tf:true).
                geometry_msgs::TransformStamped cam_to_tag;
                // Convert PoseStamped to TransformStamped (for simplicity):
                cam_to_tag.header = tag_pose_cam.header;
                cam_to_tag.child_frame_id = "tag_10_frame";  // pseudo frame name for reference tag
                cam_to_tag.transform.translation.x = tag_pose_cam.pose.position.x;
                cam_to_tag.transform.translation.y = tag_pose_cam.pose.position.y;
                cam_to_tag.transform.translation.z = tag_pose_cam.pose.position.z;
                cam_to_tag.transform.rotation = tag_pose_cam.pose.orientation;
                // Save the transform (camera->tag10)
                refCamTransform = cam_to_tag;
                haveRefTransform = true;
                ROS_INFO_THROTTLE(5.0, "Node B: Reference tag (ID 10) detected and transform recorded.");
            } catch (const std::exception &ex) {
                ROS_WARN("Node B: Failed to record reference tag transform: %s", ex.what());
            }
            // We do not add the reference tag as an object to pick, continue to next detection
            continue;
        }
        // If it's an object tag (1-9), compute its pose in base frame.
        // First, pose in camera frame -> transform to base frame:
        geometry_msgs::PoseStamped tag_pose_base;
        tf2::doTransform(tag_pose_cam, tag_pose_base, cam_to_base);
        tag_pose_base.header.frame_id = BASE_FRAME;
        // Store if not already picked
        if (picked_ids.find(tag_id) == picked_ids.end()) {
            DetectedObject obj;
            obj.id = tag_id;
            obj.pose = tag_pose_base;
            current_objects.push_back(obj);
        }
    }
    // If no reference transform yet, and reference tag not in this detection, we might wait or handle differently.
    // In this implementation, we'll simply proceed with object poses in base frame (which may shift if robot moves).
    // Once the reference tag is seen (likely when at placing table), we could adjust or use it for placement (Node A does that).
}

bool getObjectPoseService(assignment2_package::GetObjectPose::Request &req,
                          assignment2_package::GetObjectPose::Response &res) {
    // Find the first available object from current detections that is not picked
    if (current_objects.empty()) {
        ROS_WARN("Node B: No objects currently detected. Cannot provide object pose.");
        return false;
    }
    // For simplicity, just take the first in the list
    DetectedObject obj = current_objects.front();
    // Mark it as picked
    picked_ids.insert(obj.id);
    // Respond with the pose (in base frame)
    res.obj_pose = obj.pose;
    ROS_INFO("Node B: Providing object ID %d pose to requester.", obj.id);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_B");
    ros::NodeHandle nh;
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);
    
    // Subscribe to AprilTag detections
    detections_sub = nh.subscribe("/tag_detections", 1, detectionsCallback);
    // Advertise service to get an object pose
    get_obj_pose_srv = nh.advertiseService("/get_object_pose", getObjectPoseService);
    
    ROS_INFO("Node B: Detection node started, waiting for tag detections...");
    ros::spin();
    
    delete tfListenerPtr;
    return 0;
}
