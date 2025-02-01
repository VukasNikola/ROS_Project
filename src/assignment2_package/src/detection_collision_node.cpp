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

// Global pointers for TF and the PlanningSceneInterface.
tf2_ros::Buffer* g_tfBuffer = nullptr;
moveit::planning_interface::PlanningSceneInterface* g_planningScene = nullptr;
ros::Publisher feedback_pub;

// Global container to store the latest detected tag poses (keyed by tag id).
std::map<int, geometry_msgs::PoseStamped> g_tag_poses;
std::mutex g_tag_mutex;

// Global variables to store the table’s pose.
// g_original_table_pose is the “reference” (original) pose of the table – i.e. where the tag was mounted
// relative to the table’s center. g_updated_table_pose is refreshed periodically (for example, if the table
// were to move).
geometry_msgs::Pose g_original_table_pose;
geometry_msgs::Pose g_updated_table_pose;
std::mutex g_table_mutex;

/// \brief Create a collision object for a detected tag. Instead of simply using the transformed
///        detection, this function computes the (x,y) offset of the detection relative to the table’s
///        original pose and then “snaps” the object onto the table using the updated table pose.
///        The z coordinate is forced so that the object “sits” on the table.
moveit_msgs::CollisionObject createCollisionObject(int tag_id, const geometry_msgs::PoseStamped &pose_map)
{
  moveit_msgs::CollisionObject collision;
  collision.header.frame_id = "map";
  std::stringstream ss;
  ss << "object_" << tag_id;
  collision.id = ss.str();

  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose adjusted_pose;

  {
    // Compute the relative offset from the original table center.
    std::lock_guard<std::mutex> lock(g_table_mutex);
    double dx = pose_map.pose.position.x - g_original_table_pose.position.x;
    double dy = pose_map.pose.position.y - g_original_table_pose.position.y;
    // Place the object relative to the updated table pose.
    adjusted_pose.position.x = g_updated_table_pose.position.x + dx;
    adjusted_pose.position.y = g_updated_table_pose.position.y + dy;
  }

  // Use the updated table’s z position (i.e. table top) and add half the object’s height.
  double table_top = 0.0;
  {
    std::lock_guard<std::mutex> lock(g_table_mutex);
    table_top = g_updated_table_pose.position.z;
  }

  // Here we “snap” the object’s z coordinate so that its bottom touches the table.
  // (We also inflate the dimensions by 10%.) Adjust object dimensions as needed.
  if(tag_id >= 1 && tag_id <= 3)
  {
    // Represent as a cylinder (approximating a hexagonal prism).
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    double height = 0.1 * 1.1;
    double radius = 0.05 * 1.1;
    primitive.dimensions[0] = height;
    primitive.dimensions[1] = radius;
    adjusted_pose.position.z = table_top + height / 2.0;
  }
  else if(tag_id >= 4 && tag_id <= 6)
  {
    // Represent as a cube.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    double side = 0.05 * 1.1;
    primitive.dimensions[0] = side;
    primitive.dimensions[1] = side;
    primitive.dimensions[2] = side;
    adjusted_pose.position.z = table_top + side / 2.0;
  }
  else if(tag_id >= 7 && tag_id <= 9)
  {
    // Represent a triangular prism as a box.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    double width  = 0.07 * 1.1;
    double depth  = 0.035 * 1.1;
    double height = 0.05 * 1.1;
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
  
  // For simplicity, set a default orientation.
  adjusted_pose.orientation.w = 1.0;
  
  collision.primitives.push_back(primitive);
  collision.primitive_poses.push_back(adjusted_pose);
  collision.operation = collision.ADD;
  return collision;
}

/// \brief Callback for AprilTag detections.
///        Transforms the detection from the camera frame to the map frame (using the detection’s timestamp)
///        and stores the resulting pose.
void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
  for (size_t i = 0; i < msg->detections.size(); ++i)
  {
    int tag_id = msg->detections[i].id[0];

    // Get the detection in the camera frame.
    geometry_msgs::PoseStamped tag_pose_cam;
    tag_pose_cam.header = msg->header;
    tag_pose_cam.pose = msg->detections[i].pose.pose.pose;

    // Transform the detection to the map frame using its timestamp.
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
    ROS_INFO("Detected tag %d in map frame at (%.2f, %.2f, %.2f)",
             tag_id,
             tag_pose_map.pose.position.x,
             tag_pose_map.pose.position.y,
             tag_pose_map.pose.position.z);

    // Cache the detection.
    {
      std::lock_guard<std::mutex> lock(g_tag_mutex);
      g_tag_poses[tag_id] = tag_pose_map;
    }

    // Publish a brief feedback message.
    std_msgs::String fb_msg;
    std::ostringstream oss;
    oss << "Received detection for tag " << tag_id;
    fb_msg.data = oss.str();
    feedback_pub.publish(fb_msg);
  }
}

/// \brief Timer callback that refreshes the planning scene.
///        It creates (or updates) the table’s collision object and then creates the collision objects
///        for each detected tag relative to the table’s current pose.
void updateCollisionObjects(const ros::TimerEvent&)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  
  // --- Create or update the picking table collision object ---
  moveit_msgs::CollisionObject table;
  table.header.frame_id = "map";
  table.id = "pickup_table";
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  // Table dimensions are inflated by 10%.
  std::vector<double> table_dims = {0.9 * 1.1, 0.9 * 1.1, 0.9 * 1.1};
  table_primitive.dimensions = table_dims;
  geometry_msgs::Pose table_pose;
  table_pose.position.x = 7.912;
  table_pose.position.y = -3.02;
  // Here the z value is chosen so that the table’s top is at (table_dims[2]/2.0).
  table_pose.position.z = table_dims[2] / 2.0;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;
  collision_objects.push_back(table);

  {
    // Update the global updated table pose.
    std::lock_guard<std::mutex> lock(g_table_mutex);
    g_updated_table_pose = table_pose;
  }

  // --- Create collision objects for each detected tag relative to the table ---
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
  
  // Apply (or update) all collision objects.
  g_planningScene->applyCollisionObjects(collision_objects);
  ROS_INFO("Refreshed collision objects: Table plus %lu detected object(s)", collision_objects.size() - 1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_collision_node");
  ros::NodeHandle nh;
  feedback_pub = nh.advertise<std_msgs::String>("robot_feedback", 10);

  // --- Initialize the table’s reference (original) pose.
  // This represents the known (fixed) location of the table’s center where the tag is mounted.
  {
    geometry_msgs::Pose orig;
    orig.position.x = 7.912;
    orig.position.y = -3.02;
    std::vector<double> table_dims = {0.9 * 1.1, 0.9 * 1.1, 0.9 * 1.1};
    orig.position.z = table_dims[2] / 2.0;
    orig.orientation.w = 1.0;
    std::lock_guard<std::mutex> lock(g_table_mutex);
    g_original_table_pose = orig;
    g_updated_table_pose = orig;
  }

  // --- Set up TF2: Create a Buffer and TransformListener.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  g_tfBuffer = &tfBuffer;
  
  // --- Set up the MoveIt! PlanningSceneInterface.
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  g_planningScene = &planning_scene;

  // --- Subscribe to AprilTag detections.
  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagDetectionsCallback);

  // --- Create a timer to refresh collision objects (here every 0.2 sec).
  ros::Timer timer = nh.createTimer(ros::Duration(0.2), updateCollisionObjects);
  
  ros::spin();
  return 0;
}
