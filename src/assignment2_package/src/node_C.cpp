#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <set>
#include "assignment2_package/PickObject.h" // Service: request: PoseStamped target; response: bool success

// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string BASE_FRAME = "base_footprint";

// Global MoveIt interfaces
moveit::planning_interface::MoveGroupInterface* arm_group;
moveit::planning_interface::MoveGroupInterface* gripper_group;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;

// TF for coordinate transforms
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

// Track detected tags and collision objects
std::map<int, geometry_msgs::PoseStamped> detected_tags;
std::set<int> current_detection_ids;
ros::Timer update_timer;

// Add collision object to MoveIt planning scene
void addCollisionObject(const std::string& id, const shape_msgs::SolidPrimitive& primitive,
                       const geometry_msgs::Pose& pose, const std::string& frame_id) {
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    
    planning_scene_interface->applyCollisionObject(collision_object);
}

// Update collision objects for all detected tags
void updateCollisionObjects() {
    std::vector<std::string> objects_to_remove;
    
    // Update existing tags and add new ones
    for (const auto& tag_pair : detected_tags) {
        int tag_id = tag_pair.first;
        
        // Skip reference tag (ID 10)
        if (tag_id == 10) continue;
        
        try {
            // Transform to base frame if needed
            geometry_msgs::PoseStamped base_pose;
            if (tag_pair.second.header.frame_id != BASE_FRAME) {
                tfBuffer.transform(tag_pair.second, base_pose, BASE_FRAME, ros::Duration(0.1));
            } else {
                base_pose = tag_pair.second;
            }
            
            // Create collision object - simple cube for all objects
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.06; // 6cm x 6cm x 6cm cube
            primitive.dimensions[1] = 0.06;
            primitive.dimensions[2] = 0.06;
            
            // Adjust pose - lower it so cube sits on table instead of floating
            geometry_msgs::Pose obj_pose = base_pose.pose;
            obj_pose.position.z -= 0.03; // half the cube height
            
            std::string obj_id = "tag_object_" + std::to_string(tag_id);
            addCollisionObject(obj_id, primitive, obj_pose, BASE_FRAME);
            
            ROS_DEBUG("Updated collision object for tag %d", tag_id);
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform tag %d: %s", tag_id, ex.what());
        }
    }
    
    // Remove objects for tags no longer detected
    auto it = detected_tags.begin();
    while (it != detected_tags.end()) {
        if (current_detection_ids.find(it->first) == current_detection_ids.end()) {
            // Tag no longer detected, remove collision object
            objects_to_remove.push_back("tag_object_" + std::to_string(it->first));
            ROS_INFO("Removing collision object for tag %d (no longer detected)", it->first);
            it = detected_tags.erase(it);
        } else {
            ++it;
        }
    }
    
    // Remove collision objects
    if (!objects_to_remove.empty()) {
        planning_scene_interface->removeCollisionObjects(objects_to_remove);
    }
}

// Timer callback for regular updates
void timerCallback(const ros::TimerEvent&) {
    updateCollisionObjects();
}

// AprilTag detection callback
void detectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    current_detection_ids.clear();
    
    // Process each detected tag
    for (const auto& tag : msg->detections) {
        if (tag.id.empty()) continue;
        
        int tag_id = tag.id[0];
        current_detection_ids.insert(tag_id);
        
        // Store the detection
        geometry_msgs::PoseStamped camera_to_tag;
        camera_to_tag.header = tag.pose.header;
        camera_to_tag.pose = tag.pose.pose.pose;
        
        detected_tags[tag_id] = camera_to_tag;
    }
}

// Service callback for pick operation
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                       assignment2_package::PickObject::Response &res) {
    ROS_INFO("Node C: PickObject service called.");
    
    geometry_msgs::PoseStamped target_pose = req.target;
    geometry_msgs::Pose obj_pose = target_pose.pose;
    
    // Open gripper first
    gripper_group->setNamedTarget("open");
    gripper_group->move();
    
    // Add table collision object (only once)
    static bool table_added = false;
    if (!table_added) {
        shape_msgs::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::SolidPrimitive::BOX;
        table_shape.dimensions = {1.0, 1.0, 0.05}; // 1m x 1m x 5cm table
        
        geometry_msgs::Pose table_pose;
        table_pose.position.x = 0.7; // in front of robot
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.72; // table height
        table_pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));
        
        addCollisionObject("table", table_shape, table_pose, BASE_FRAME);
        table_added = true;
        ROS_INFO("Node C: Added table collision object.");
    }
    
    // Plan approach pose above object
    geometry_msgs::PoseStamped approach_pose;
    approach_pose.header.frame_id = BASE_FRAME;
    approach_pose.pose = obj_pose;
    approach_pose.pose.position.z += 0.10; // 10cm above
    
    // Set downward orientation for top-down grasp
    tf2::Quaternion down_quat;
    down_quat.setRPY(M_PI, 0, 0); // 180 degree rotation around X
    approach_pose.pose.orientation = tf2::toMsg(down_quat);
    
    // Plan and execute approach
    arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
        ROS_ERROR("Node C: Cannot plan approach to object. Aborting pick.");
        res.success = false;
        return true;
    }
    
    ROS_INFO("Node C: Executing approach to object...");
    arm_group->execute(plan);
    
    // Linear descent to object
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(approach_pose.pose);
    
    geometry_msgs::Pose grasp_pose = approach_pose.pose;
    grasp_pose.position.z -= 0.10; // down to object
    waypoints.push_back(grasp_pose);
    
    moveit_msgs::RobotTrajectory cart_traj;
    double fraction = arm_group->computeCartesianPath(waypoints, 0.01, 0.0, cart_traj);
    
    if (fraction > 0.8) { // Accept if 80% of path is planned
        ROS_INFO("Node C: Executing descent to grasp object...");
        arm_group->execute(cart_traj);
    } else {
        ROS_ERROR("Node C: Cannot plan descent path. Aborting pick.");
        res.success = false;
        return true;
    }
    
    // Close gripper to grasp
    ROS_INFO("Node C: Closing gripper to grasp object.");
    gripper_group->setNamedTarget("close");
    gripper_group->move();
    
    // Remove the picked object from collision objects
    // Find and remove the collision object for this pose
    std::vector<std::string> objects_to_remove;
    for (const auto& tag_pair : detected_tags) {
        geometry_msgs::PoseStamped tag_pose = tag_pair.second;
        
        // Simple distance check to find which object was picked
        double dist = sqrt(pow(tag_pose.pose.position.x - obj_pose.position.x, 2) +
                          pow(tag_pose.pose.position.y - obj_pose.position.y, 2) +
                          pow(tag_pose.pose.position.z - obj_pose.position.z, 2));
        
        if (dist < 0.1) { // Within 10cm - likely the same object
            objects_to_remove.push_back("tag_object_" + std::to_string(tag_pair.first));
            ROS_INFO("Node C: Removing collision object for picked tag %d", tag_pair.first);
            break;
        }
    }
    
    if (!objects_to_remove.empty()) {
        planning_scene_interface->removeCollisionObjects(objects_to_remove);
    }
    
    // Lift object back up
    ROS_INFO("Node C: Lifting object...");
    arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
    if (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        arm_group->execute(plan);
    }
    
    // Move to safe travel pose
    ROS_INFO("Node C: Moving to safe travel pose...");
    arm_group->setNamedTarget("folded");
    arm_group->move();
    
    ROS_INFO("Node C: Pick operation completed successfully.");
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_C");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Parameters
    double update_rate = pnh.param("update_rate", 10.0); // 10Hz updates
    
    // Initialize TF
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    
    // Initialize MoveIt interfaces
    static moveit::planning_interface::MoveGroupInterface arm_move_group(PLANNING_GROUP_ARM);
    static moveit::planning_interface::MoveGroupInterface gripper_move_group(PLANNING_GROUP_GRIPPER);
    static moveit::planning_interface::PlanningSceneInterface psi;
    
    arm_group = &arm_move_group;
    gripper_group = &gripper_move_group;
    planning_scene_interface = &psi;
    
    // Set planning parameters
    arm_group->setPlanningTime(10.0);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);
    
    // Subscribe to AprilTag detections for collision object management
    ros::Subscriber detections_sub = nh.subscribe("/tag_detections", 1, detectionsCallback);
    
    // Set up timer for regular collision object updates
    update_timer = nh.createTimer(ros::Duration(1.0/update_rate), timerCallback);
    
    // Start the pick service
    ros::ServiceServer service = nh.advertiseService("/pick_object", pickObjectCallback);
    
    ROS_INFO("Node C: Ready to manage collision objects and handle pick requests.");
    
    ros::spin();
    
    delete tfListener;
    return 0;
}