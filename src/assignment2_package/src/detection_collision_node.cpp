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

// Global pointers for TF and PlanningSceneInterface.
tf2_ros::Buffer* g_tfBuffer = nullptr;
moveit::planning_interface::PlanningSceneInterface* g_planningScene = nullptr;

// Create collision objects for objects based on tag IDs.
moveit_msgs::CollisionObject createCollisionObject(int tag_id, const geometry_msgs::PoseStamped &pose_robot)
{
  moveit_msgs::CollisionObject collision;
  collision.header.frame_id = "base_link";
  std::stringstream ss;
  ss << "object_" << tag_id;
  collision.id = ss.str();

  shape_msgs::SolidPrimitive primitive;
  // Apply a safety margin of 10%.
  if(tag_id >= 1 && tag_id <= 3) {
    // Hexagonal prism → approximate as a cylinder (h: 0.1, r: 0.05)
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.1 * 1.1;  // height
    primitive.dimensions[1] = 0.05 * 1.1; // radius
  } else if(tag_id >= 4 && tag_id <= 6) {
    // Cube (l: 0.05)
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    double side = 0.05 * 1.1;
    primitive.dimensions[0] = side;
    primitive.dimensions[1] = side;
    primitive.dimensions[2] = side;
  } else if(tag_id >= 7 && tag_id <= 9) {
    // Triangular prism → approximate as a box (b: 0.07, h: 0.035, L: 0.05)
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.07 * 1.1;
    primitive.dimensions[1] = 0.035 * 1.1;
    primitive.dimensions[2] = 0.05 * 1.1;
  } else {
    ROS_WARN("Tag id %d not recognized for an object", tag_id);
    return collision;
  }
  
  collision.primitives.push_back(primitive);
  collision.primitive_poses.push_back(pose_robot.pose);
  collision.operation = collision.ADD;
  return collision;
}

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
  for (size_t i = 0; i < msg->detections.size(); ++i)
  {
    int tag_id = msg->detections[i].id[0];
    // Process only object tags (IDs 1–9).
    if(tag_id < 1 || tag_id > 9)
      continue;

    // Transform the detection from the camera frame to base_link.
    geometry_msgs::PoseStamped tag_pose_cam;
    tag_pose_cam.header = msg->header;
    tag_pose_cam.pose = msg->detections[i].pose.pose.pose;
    
    geometry_msgs::PoseStamped tag_pose_robot;
    try {
      geometry_msgs::TransformStamped transformStamped =
          g_tfBuffer->lookupTransform("base_link", tag_pose_cam.header.frame_id,
                                      ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(tag_pose_cam, tag_pose_robot, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("TF transform error: %s", ex.what());
      continue;
    }
    
    ROS_INFO("Detected tag %d at (%.2f, %.2f, %.2f)", tag_id,
             tag_pose_robot.pose.position.x,
             tag_pose_robot.pose.position.y,
             tag_pose_robot.pose.position.z);
    
    // Create and add the collision object for the detected object.
    moveit_msgs::CollisionObject obj = createCollisionObject(tag_id, tag_pose_robot);
    std::vector<moveit_msgs::CollisionObject> objs;
    objs.push_back(obj);
    g_planningScene->applyCollisionObjects(objs);

    // Publish the candidate object pose for picking.
    static ros::Publisher obj_pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("object_pick_pose", 10);
    obj_pose_pub.publish(tag_pose_robot);
    break;  // Process one object for now.
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_collision_node");
  ros::NodeHandle nh;
  
  // Set up TF buffer and listener.
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  g_tfBuffer = &tfBuffer;

  // Set up MoveIt! PlanningSceneInterface.
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  g_planningScene = &planning_scene;
  
  // --- Add the Table as a Collision Object ---
  // The table is static in the environment. Its frame is set to "map".
  moveit_msgs::CollisionObject table;
  table.header.frame_id = "map";
  table.id = "pickup_table";
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  // Table dimensions (0.9 m, with 10% margin)
  std::vector<double> table_dims = {0.9 * 1.1, 0.9 * 1.1, 0.9 * 1.1};
  table_primitive.dimensions = table_dims;
  geometry_msgs::Pose table_pose;
  // Adjust the pose so that the table sits at the proper location in "map".
  table_pose.position.x = 1.0;
  table_pose.position.y = 0.0;
  // Place table top at a desired height; here we assume the table is on the ground.
  table_pose.position.z = table_dims[2] / 2.0;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;
  planning_scene.applyCollisionObject(table);
  
  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagDetectionsCallback);
  
  ros::spin();
  return 0;
}
