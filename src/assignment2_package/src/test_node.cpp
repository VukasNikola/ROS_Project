#include "ros/ros.h"
#include <sstream>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <boost/bind.hpp>
#include <map>
#include <set>

// MoveIt includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

tf2_ros::TransformBroadcaster *brPtr;
tf2_ros::Buffer *tfBufferPtr;
moveit::planning_interface::PlanningSceneInterface *planningScenePtr;

// Store detected tag IDs and their poses
std::map<int, geometry_msgs::PoseStamped> detected_tags;
std::set<int> current_detection_ids;
ros::Timer update_timer;

void updateCollisionObjects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<std::string> object_ids_to_remove;
    
    // Get current time for all transforms
    ros::Time current_time = ros::Time::now();
    
    // Update existing tags and add new ones
    for (const auto& tag_pair : detected_tags)
    {
        int tag_id = tag_pair.first;
        geometry_msgs::PoseStamped camera_pose = tag_pair.second;
        
        try
        {
            // Transform to base_link (or planning frame)
            geometry_msgs::PoseStamped base_pose;
            tfBufferPtr->transform(camera_pose, base_pose, "base_link", ros::Duration(0.1));
            
            // Create collision object
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_link";
            collision_object.header.stamp = current_time;
            collision_object.id = "tag_sphere_" + std::to_string(tag_id);
            
            // Define sphere primitive
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = 0.05; // 5cm radius sphere
            
            // Set pose
            geometry_msgs::Pose pose;
            pose.position = base_pose.pose.position;
            pose.orientation = base_pose.pose.orientation;
            
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = collision_object.ADD;
            
            collision_objects.push_back(collision_object);
            
            // Update transform broadcast
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = current_time;
            transformStamped.header.frame_id = "base_link";
            transformStamped.child_frame_id = "tag_" + std::to_string(tag_id);
            transformStamped.transform.translation.x = base_pose.pose.position.x;
            transformStamped.transform.translation.y = base_pose.pose.position.y;
            transformStamped.transform.translation.z = base_pose.pose.position.z;
            transformStamped.transform.rotation = base_pose.pose.orientation;
            
            brPtr->sendTransform(transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not transform tag %d: %s", tag_id, ex.what());
        }
    }
    
    // Apply collision objects to planning scene
    if (!collision_objects.empty())
    {
        planningScenePtr->addCollisionObjects(collision_objects);
        ROS_DEBUG("Updated %zu collision objects", collision_objects.size());
    }
}

void timerCallback(const ros::TimerEvent&)
{
    updateCollisionObjects();
}

void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    ROS_DEBUG("Detection callback called with %zu detections", msg->detections.size());
    
    // Clear current detection set
    current_detection_ids.clear();
    
    // Process each detected tag
    for (const auto &tag : msg->detections)
    {
        if (tag.id.empty()) continue;
        
        int tag_id = tag.id[0];
        current_detection_ids.insert(tag_id);
        
        // Store the detection in camera frame
        geometry_msgs::PoseStamped camera_to_tag;
        camera_to_tag.header = tag.pose.header;
        camera_to_tag.pose = tag.pose.pose.pose;
        
        detected_tags[tag_id] = camera_to_tag;
        
        ROS_DEBUG("Updated tag %d pose", tag_id);
    }
    
    // Remove tags that are no longer detected
    std::vector<std::string> objects_to_remove;
    auto it = detected_tags.begin();
    while (it != detected_tags.end())
    {
        if (current_detection_ids.find(it->first) == current_detection_ids.end())
        {
            // Tag no longer detected, remove collision object
            objects_to_remove.push_back("tag_sphere_" + std::to_string(it->first));
            ROS_INFO("Removing tag %d (no longer detected)", it->first);
            it = detected_tags.erase(it);
        }
        else
        {
            ++it;
        }
    }
    
    // Remove collision objects for tags no longer detected
    if (!objects_to_remove.empty())
    {
        planningScenePtr->removeCollisionObjects(objects_to_remove);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_manipulation_moveit");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Parameters
    double sphere_radius = pnh.param("sphere_radius", 0.05); // Default 5cm radius
    double update_rate = pnh.param("update_rate", 10.0);     // Default 10Hz
    
    ROS_INFO("Starting AprilTag MoveIt node with %f Hz update rate", update_rate);
    
    // Initialize TF components
    static tf2_ros::TransformBroadcaster br;
    tf2_ros::Buffer tfBuffer(ros::Duration(10.0)); // 10 second buffer
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    // Initialize MoveIt planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Set global pointers
    brPtr = &br;
    tfBufferPtr = &tfBuffer;
    planningScenePtr = &planning_scene_interface;
    
    // Wait for TF to be ready
    ROS_INFO("Waiting for TF to be ready...");
    ros::Duration(2.0).sleep(); // Give TF time to initialize
    
    // Set up message filter for synchronized processing
    message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> sub(nh, "tag_detections", 10);
    
    tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray> *mf;
    mf = new tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray>(
        sub, tfBuffer, "base_link", 10, nh);
    
    mf->registerCallback(boost::bind(&detectionCallback, _1));
    
    // Set up timer for regular updates (10Hz)
    update_timer = nh.createTimer(ros::Duration(1.0/update_rate), timerCallback);
    
    ROS_INFO("AprilTag MoveIt node initialized. Listening for detections...");
    
    ros::spin();
    
    // Cleanup
    delete mf;
    
    return 0;
}