#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "assignment2_package/PickObject.h"
#include "assignment2_package/ObjectPose.h"
#include "assignment2_package/ObjectPoseArray.h"
#include "assignment2_package/GetObjectPose.h"
#include "gripper_control.h"
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <boost/variant.hpp>
#include <map>
#include <set>
#include <cstdint>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm_torso";  // Changed from "arm" to "arm_torso"
static const std::string PLANNING_GROUP_GRIPPER = "gripper"; // Not needed maybe, due to the .h
static const std::string BASE_FRAME = "base_footprint";

// Globals
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group; // Same as planning group
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
ros::ServiceClient get_obj_pose_client;

// Keep track of which collision objects are currently in the scene
std::map<int, geometry_msgs::PoseStamped> detected_tags;
std::map<int, bool> persistent_tables; // Track which tables have been created

// Helper: add or update a collision object
void addCollisionObject(const std::string &id,
                        const shape_msgs::SolidPrimitive &primitive,
                        const geometry_msgs::Pose &pose,
                        const std::string &frame_id)
{
  moveit_msgs::CollisionObject co;
  co.id = id;
  co.header.frame_id = frame_id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface->applyCollisionObject(co);
}

// Helper: add a single‐pose, scaled mesh collision object
void addMeshCollisionObject(const std::string &id,
                            const std::string &mesh_resource,
                            const geometry_msgs::Pose &pose,
                            const std::string &frame_id,
                            const Eigen::Vector3d &scale)
{
  moveit_msgs::CollisionObject co;
  co.id = id;
  co.header.frame_id = frame_id;

  // Load & scale the mesh in one go:
  // (this overload applies your scale to X, Y, Z axes)
  shapes::Mesh *m = shapes::createMeshFromResource(mesh_resource, scale);

  // Convert to the ShapeMsg variant and extract the mesh
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  const shape_msgs::Mesh &mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Populate and apply
  co.meshes.push_back(mesh_msg);
  co.mesh_poses.push_back(pose);
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface->applyCollisionObjects({co});
}

// Subscriber callback: update scene from ObjectPoseArray
void objectPosesCallback(const assignment2_package::ObjectPoseArray::ConstPtr &msg)
{
  std::set<int> seen_ids;
  ROS_INFO("Processing object poses callback");

  // Precompute mesh URI and scale vector
  std::string pkg_path = ros::package::getPath("tiago_iaslab_simulation");
  std::string mesh_uri = "file://" + pkg_path + "/meshes/triangle_centered.stl";
  Eigen::Vector3d prism_scale(
      0.5, // X-scale
      0.5, // Y-scale
      0.5  // Z-scale
  );

  for (const auto &entry : msg->objects)
  {
    int tag_id = entry.id;
    seen_ids.insert(tag_id);

    geometry_msgs::Pose obj_pose = entry.pose.pose;

    // Mesh for tags 7–9 (using your working version)
    if (tag_id >= 7 && tag_id <= 9)
    {
      // 1) Get tag orientation
      tf2::Quaternion tag_q;
      tf2::fromMsg(entry.pose.pose.orientation, tag_q);

      // 2) Correction: 180° about X, then 90° about Z
      tf2::Quaternion corr_q;
      corr_q.setRPY(M_PI, 0.0, M_PI / 2.0);

      // 3) Combine & write back
      tf2::Quaternion final_q = tag_q * corr_q;
      obj_pose.orientation = tf2::toMsg(final_q);

      // 4) Add mesh with scale
      std::string obj_id = "tag_mesh_" + std::to_string(tag_id);
      addMeshCollisionObject(obj_id,
                             mesh_uri,
                             obj_pose,
                             BASE_FRAME,
                             prism_scale);

      detected_tags[tag_id] = entry.pose;
      continue;
    }

    // Primitives for all other tags - adjust Z position based on object height
    shape_msgs::SolidPrimitive primitive;
    double height_offset = 0.0;

    if (tag_id >= 1 && tag_id <= 3)
    {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions = {0.1, 0.025};           // height, radius
      height_offset = primitive.dimensions[0] / 2.0; // half of cylinder height
    }
    else
    {
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.05, 0.05, 0.05};     // x, y, z
      height_offset = primitive.dimensions[2] / 2.0; // half of box height
    }

    // Adjust Z position: move down by half the object height so bottom sits on surface
    obj_pose.position.z -= height_offset;

    std::string obj_id = "tag_object_" + std::to_string(tag_id);
    addCollisionObject(obj_id, primitive, obj_pose, BASE_FRAME);
    detected_tags[tag_id] = entry.pose;
  }

  // Removal logic
  std::vector<std::string> to_remove;
  for (auto it = detected_tags.begin(); it != detected_tags.end();)
  {
    int id = it->first;
    if (!seen_ids.count(id))
    {
      to_remove.push_back((id >= 7 && id <= 9)
                              ? "tag_mesh_" + std::to_string(id)
                              : "tag_object_" + std::to_string(id));
      it = detected_tags.erase(it);
    }
    else
      ++it;
  }
  if (!to_remove.empty())
  {
    planning_scene_interface->removeCollisionObjects(to_remove);
    ROS_INFO("Removed %zu old collision objects", to_remove.size());
  }
}

// Service callback for pick operation
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  ROS_INFO("PickObject service called - executing proper pick sequence");
  
  try {
    // 1. Get the target object ID from the request
    int id = req.target_id;
    // choose mesh names for tags 7–9, otherwise object names
    std::string object_id = (id >= 7 && id <= 9)
        ? "tag_mesh_" + std::to_string(id)
        : "tag_object_" + std::to_string(id);
    
    // 2. Create a vector of grasps
    std::vector<moveit_msgs::Grasp> grasps;
    
    // 3. Generate grasp poses (this is a simplified version - you'd want multiple grasps)
    moveit_msgs::Grasp grasp;
    
    // Set the grasp pose (approach from above)
    grasp.grasp_pose.header.frame_id = BASE_FRAME;
    grasp.grasp_pose.pose = req.target.pose;
    
    // Adjust the grasp pose - typically approach from above
    grasp.grasp_pose.pose.position.z += 0.1; // Approach from 10cm above
    
    // Set gripper orientation to point downward
    // tf2::Quaternion grasp_orientation;
    // grasp_orientation.setRPY(0, M_PI/2, 0); // Point gripper downward
    // grasp.grasp_pose.pose.orientation = tf2::toMsg(grasp_orientation);
    
    // Set the pre-grasp approach
    grasp.pre_grasp_approach.direction.header.frame_id = BASE_FRAME;
    grasp.pre_grasp_approach.direction.vector.z = -1.0; // Approach downward
    grasp.pre_grasp_approach.min_distance = 0.05;
    grasp.pre_grasp_approach.desired_distance = 0.1;
    
    // Set the post-grasp retreat
    grasp.post_grasp_retreat.direction.header.frame_id = BASE_FRAME;
    grasp.post_grasp_retreat.direction.vector.z = 1.0; // Retreat upward
    grasp.post_grasp_retreat.min_distance = 0.05;
    grasp.post_grasp_retreat.desired_distance = 0.1;
    
    // Set the pre-grasp posture (open gripper)
    grasp.pre_grasp_posture.joint_names.push_back("gripper_left_finger_joint");
    grasp.pre_grasp_posture.joint_names.push_back("gripper_right_finger_joint");
    grasp.pre_grasp_posture.points.resize(1);
    grasp.pre_grasp_posture.points[0].positions.resize(2);
    grasp.pre_grasp_posture.points[0].positions[0] = 0.04; // Open position
    grasp.pre_grasp_posture.points[0].positions[1] = 0.04; // Open position
    grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(2.0);
    
    // Set the grasp posture (close gripper)
    grasp.grasp_posture.joint_names.push_back("gripper_left_finger_joint");
    grasp.grasp_posture.joint_names.push_back("gripper_right_finger_joint");
    grasp.grasp_posture.points.resize(1);
    grasp.grasp_posture.points[0].positions.resize(2);
    grasp.grasp_posture.points[0].positions[0] = 0.01; // Closed position
    grasp.grasp_posture.points[0].positions[1] = 0.01; // Closed position
    grasp.grasp_posture.points[0].time_from_start = ros::Duration(2.0);
    
    // Set grasp quality metrics
    grasp.grasp_quality = 1.0;
    
    grasps.push_back(grasp);
    
    // 4. Execute the pick operation
    moveit::core::MoveItErrorCode pick_result = arm_group->pick(object_id, grasps);
    
    if (pick_result == moveit::core::MoveItErrorCode::SUCCESS) {
      ROS_INFO("Pick operation successful!");
      res.success = true;
    } else {
      ROS_ERROR("Pick operation failed with error code: %d", pick_result.val);
      res.success = false;
    }
    
  } catch (const std::exception &e) {
    ROS_ERROR("Exception in pickObjectCallback: %s", e.what());
    res.success = false;
  }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_C");
  ros::NodeHandle nh;
  
  // Use AsyncSpinner to process service callbacks and MoveIt
  ros::AsyncSpinner spinner(2); // 2 threads should be sufficient
  spinner.start();

  // TF
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  // MoveIt interfaces with error handling and debug info
  try {
    // Debug: Print available planning groups
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    const std::vector<std::string>& group_names = kinematic_model->getJointModelGroupNames();

    ROS_INFO("Available planning groups:");
    for(const std::string& group_name : group_names) {
        ROS_INFO("  - %s", group_name.c_str());
    }
    
    ROS_INFO("Initializing MoveGroupInterface with group: %s", PLANNING_GROUP_ARM.c_str());
    static moveit::planning_interface::MoveGroupInterface arm_mg(PLANNING_GROUP_ARM);
    
    ROS_INFO("Initializing gripper group: %s", PLANNING_GROUP_GRIPPER.c_str());
    static moveit::planning_interface::MoveGroupInterface gripper_mg(PLANNING_GROUP_GRIPPER);
    
    static moveit::planning_interface::PlanningSceneInterface psi;
    
    arm_group = &arm_mg;
    gripper_group = &gripper_mg;
    planning_scene_interface = &psi;

    arm_group->setPlanningTime(10.0);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    ROS_INFO("MoveIt interfaces initialized successfully");
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to initialize MoveIt interfaces: %s", e.what());
    return -1;
  }

  // Subscribe & service as before
  ros::Subscriber object_sub = nh.subscribe("object_poses", 1, objectPosesCallback);
  /// Service client to Node B's get_object_pose
  get_obj_pose_client = nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
  ros::ServiceServer pick_service = nh.advertiseService("pick_object", pickObjectCallback);

  ROS_INFO("Node C: Ready with arm_torso group for extended reach");

  ros::waitForShutdown();
  delete tfListener;
  return 0;
}