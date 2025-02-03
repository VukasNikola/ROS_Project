#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <sstream>
#include <vector>
#include <std_msgs/String.h>
#include <map>
#include <mutex>

// Global pointers and variables.
tf2_ros::Buffer* g_tfBuffer = nullptr;
moveit::planning_interface::PlanningSceneInterface* g_planningScene = nullptr;
ros::Publisher feedback_pub;

std::map<int, geometry_msgs::PoseStamped> g_tag_poses;
std::mutex g_tag_mutex;

geometry_msgs::Pose g_original_table_pose;
geometry_msgs::Pose g_updated_table_pose;
std::mutex g_table_mutex;

// Constants for the pickup table dimensions.
const double PICKUP_TABLE_WIDTH  = 0.9;
const double PICKUP_TABLE_DEPTH  = 0.9;
const double PICKUP_TABLE_HEIGHT = 0.9;  // Total height of the table

/**
 * @brief Create a collision object based on the detected tag id.
 *
 * - For tags 1–9, the object's (x,y) is adjusted relative to the pickup table.
 *   A scaling factor is applied so that the offset isn’t too large.
 * - For tag 10, a collision object identical in size to the pickup table is created,
 *   but its position is fixed: it is placed 1.0 m in the positive y direction relative
 *   to the pickup table.
 *
 * @param tag_id The id of the detected tag.
 * @param pose_map The detected tag pose in the "map" frame.
 * @return A moveit_msgs::CollisionObject for use in the planning scene.
 */
moveit_msgs::CollisionObject createCollisionObject(int tag_id, const geometry_msgs::PoseStamped &pose_map)
{
  moveit_msgs::CollisionObject collision;
  collision.header.frame_id = "map";

  // Create a unique id for the object.
  std::stringstream ss;
  ss << "object_" << tag_id;
  collision.id = ss.str();

  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose adjusted_pose;

  if (tag_id == 10)
  {
    // ----- Tag 10: Create a table identical to the pickup table,
    // but shifted by 1 meter along the y-axis.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = PICKUP_TABLE_WIDTH;
    primitive.dimensions[1] = PICKUP_TABLE_DEPTH;
    primitive.dimensions[2] = PICKUP_TABLE_HEIGHT;

    {
      std::lock_guard<std::mutex> lock(g_table_mutex);
      adjusted_pose.position.x = g_updated_table_pose.position.x;
      adjusted_pose.position.y = g_updated_table_pose.position.y + 1.0; // shift by 1 meter in y
      adjusted_pose.position.z = PICKUP_TABLE_HEIGHT / 2.0;
    }
    adjusted_pose.orientation.w = 1.0;
  }
  else
  {
    // ----- For tags 1–9:
    // Compute the offset from the table's original pose.
    // Adjust the scaling factor as needed. (Set scale to 0.0 to ignore the offset.)
    double scale = 0.0;
    double dx = (pose_map.pose.position.x - g_original_table_pose.position.x) * scale;
    double dy = (pose_map.pose.position.y - g_original_table_pose.position.y) * scale;
    
    {
      std::lock_guard<std::mutex> lock(g_table_mutex);
      adjusted_pose.position.x = g_updated_table_pose.position.x + dx;
      adjusted_pose.position.y = g_updated_table_pose.position.y + dy;
    }
    
    // Compute the top of the pickup table.
    double table_top = 0.0;
    {
      std::lock_guard<std::mutex> lock(g_table_mutex);
      // g_updated_table_pose.position.z is the center; add half the table height.
      table_top = g_updated_table_pose.position.z + PICKUP_TABLE_HEIGHT / 2.0;
    }
    
    if (tag_id >= 1 && tag_id <= 3)
    {
      // Blue Hexagon tags (IDs 1-3): Create a cylinder.
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      double obj_height = 0.1;
      double obj_radius = 0.05;
      primitive.dimensions[0] = obj_height;
      primitive.dimensions[1] = obj_radius;
      adjusted_pose.position.z = table_top + obj_height / 2.0;
    }
    else if (tag_id >= 4 && tag_id <= 6)
    {
      // Red Cube tags (IDs 4-6): Create a small box.
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      double side = 0.05;
      primitive.dimensions[0] = side;
      primitive.dimensions[1] = side;
      primitive.dimensions[2] = side;
      adjusted_pose.position.z = table_top + side / 2.0;
    }
    else if (tag_id >= 7 && tag_id <= 9)
    {
      // Green Triangle tags (IDs 7-9): Create a box with different proportions.
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      double width  = 0.07;
      double depth  = 0.035;
      double height = 0.05;
      primitive.dimensions[0] = width;
      primitive.dimensions[1] = depth;
      primitive.dimensions[2] = height;
      adjusted_pose.position.z = table_top + height / 2.0;
    }
    else
    {
      ROS_WARN("Tag id %d not recognized for an object", tag_id);
      return collision;
    }
    adjusted_pose.orientation.w = 1.0;
  }
  
  collision.primitives.push_back(primitive);
  collision.primitive_poses.push_back(adjusted_pose);
  collision.operation = collision.ADD;
  
  return collision;
}

/**
 * @brief Callback for AprilTag detections.
 *
 * Transforms each detection from its camera frame to the map frame and stores it.
 */
void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
  for (size_t i = 0; i < msg->detections.size(); ++i)
  {
    int tag_id = msg->detections[i].id[0];

    geometry_msgs::PoseStamped tag_pose_cam;
    tag_pose_cam.header = msg->header;
    tag_pose_cam.pose = msg->detections[i].pose.pose.pose;

    geometry_msgs::PoseStamped tag_pose_map;
    try {
      geometry_msgs::TransformStamped transformStamped =
          g_tfBuffer->lookupTransform("map", tag_pose_cam.header.frame_id,
                                       tag_pose_cam.header.stamp, ros::Duration(1.0));
      tf2::doTransform(tag_pose_cam, tag_pose_map, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("TF transform error: %s", ex.what());
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(g_tag_mutex);
      g_tag_poses[tag_id] = tag_pose_map;
    }

    std_msgs::String fb_msg;
    std::ostringstream oss;
    oss << "Received detection for tag " << tag_id;
    fb_msg.data = oss.str();
    feedback_pub.publish(fb_msg);
  }
}

/**
 * @brief Timer callback to update collision objects in the planning scene.
 *
 * This function continuously updates the pickup table collision object (which remains static)
 * and then creates/updates collision objects for each detected tag.
 */
void updateCollisionObjects(const ros::TimerEvent&)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  
  // ----- Create the pickup table collision object.
  moveit_msgs::CollisionObject table;
  table.header.frame_id = "map";
  table.id = "pickup_table";
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  std::vector<double> table_dims = {PICKUP_TABLE_WIDTH, PICKUP_TABLE_DEPTH, PICKUP_TABLE_HEIGHT};
  table_primitive.dimensions = table_dims;
  
  geometry_msgs::Pose table_pose;
  table_pose.position.x = 7.912;
  table_pose.position.y = -3.02;
  table_pose.position.z = PICKUP_TABLE_HEIGHT / 2.0;
  table_pose.orientation.w = 1.0;
  
  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;
  collision_objects.push_back(table);

  // Update the global table pose.
  {
    std::lock_guard<std::mutex> lock(g_table_mutex);
    g_updated_table_pose = table_pose;
  }

  // ----- Create collision objects for each detected tag.
  {
    std::lock_guard<std::mutex> lock(g_tag_mutex);
    for (const auto &tag_pair : g_tag_poses)
    {
      int tag_id = tag_pair.first;
      const geometry_msgs::PoseStamped &pose_map = tag_pair.second;
      moveit_msgs::CollisionObject obj = createCollisionObject(tag_id, pose_map);
      if (!obj.id.empty())
        collision_objects.push_back(obj);
    }
  }
  
  // Apply (update) the collision objects in the planning scene.
  g_planningScene->applyCollisionObjects(collision_objects);
  ROS_INFO("Refreshed collision objects: Table plus %lu detected object(s)",
           collision_objects.size() - 1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_collision_node");
  ros::NodeHandle nh;
  feedback_pub = nh.advertise<std_msgs::String>("robot_feedback", 10);

  // Initialize the table pose (both original and updated) with the pickup table's known pose.
  {
    geometry_msgs::Pose orig;
    orig.position.x = 7.912;
    orig.position.y = -3.02;
    orig.position.z = PICKUP_TABLE_HEIGHT / 2.0;
    orig.orientation.w = 1.0;
    std::lock_guard<std::mutex> lock(g_table_mutex);
    g_original_table_pose = orig;
    g_updated_table_pose = orig;
  }

  // Set up TF listener.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  g_tfBuffer = &tfBuffer;
  
  // Set up the planning scene interface.
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  g_planningScene = &planning_scene;

  // Subscribe to AprilTag detections.
  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagDetectionsCallback);
  
  // Set up a timer to update collision objects constantly.
  ros::Timer timer = nh.createTimer(ros::Duration(0.2), updateCollisionObjects);
  
  ros::spin();
  return 0;
}
