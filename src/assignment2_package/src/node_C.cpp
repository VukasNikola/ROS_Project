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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm_torso";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string BASE_FRAME = "base_footprint";

// Globals
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
tf2_ros::Buffer *tfBuffer;
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
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  ROS_INFO("PickObject service called - executing pick sequence");
  try
  {
    // Initialize gripper controller (assuming "gripper" is the planning group name)
    gripper_control::GripperController gripper("gripper");
    
    // Register named targets for gripper
    gripper.registerNamedTarget("open", 0.044);    // Max opening
    gripper.registerNamedTarget("closed", 0.0);    // Fully closed
    
    // Open gripper before approaching
    ROS_INFO("Opening gripper before approach");
    gripper.open();
    
    // Set the end effector link
    arm_group->setEndEffectorLink("gripper_grasping_frame");

    // Get the target pose from the request
    geometry_msgs::Pose target_pose = req.target.pose;

    // STEP 1: Calculate the above_pose and object_height based on the ID
    geometry_msgs::Pose above_pose;
    tf2::Quaternion final_orientation_q;
    double object_height;
    int id = req.target_id;

    if (id >= 1 && id <= 3)
    {
      // Case 1: For cylinders
      ROS_INFO("Using torso_fixed link orientation for object ID %d", id);
      object_height = 0.1;

      // Get torso_fixed_link orientation
      geometry_msgs::TransformStamped torso_transform;
      try
      {
        torso_transform = tfBuffer->lookupTransform(req.target.header.frame_id, "torso_fixed_link", ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(torso_transform.transform.rotation, final_orientation_q);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_ERROR("Could not get transform: %s", ex.what());
        res.success = false;
        return true;
      }

      // Start from target position and move up 0.2m in world Z
      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 4 && id <= 6)
    {
      // Case 2: For cubes
      ROS_INFO("Using target_pose orientation for object ID %d", id);
      object_height = 0.05;

      // Use target orientation
      tf2::fromMsg(target_pose.orientation, final_orientation_q);

      // Start from target position and move up 0.2m in world Z
      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 7 && id <= 9)
    {
      // Case 3: For prism on slope
      ROS_INFO("Handling prism on slope for ID %d", id);
      object_height = 0.035;

      // Get the target orientation
      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);

      // Create 45-degree rotation around local X-axis
      tf2::Quaternion q_45_deg_x;
      q_45_deg_x.setRPY(M_PI / 4, 0, 0);

      // Apply the rotation: first target orientation, then 45-deg rotation
      tf2::Quaternion temp_orientation_q = target_q * q_45_deg_x;
      temp_orientation_q.normalize();

      // Move 0.2m along the rotated Z-axis to get above position
      tf2::Vector3 z_offset_local(0, 0, 0.2);
      tf2::Vector3 z_offset_world = tf2::quatRotate(temp_orientation_q, z_offset_local);

      // Apply the offset to get above position
      above_pose.position = target_pose.position;
      above_pose.position.x += z_offset_world.x();
      above_pose.position.y += z_offset_world.y();
      above_pose.position.z += z_offset_world.z();

      // Now apply -90 degree rotation around Z-axis for reachable gripper orientation
      tf2::Quaternion q_neg_90_deg_z;
      q_neg_90_deg_z.setRPY(0, 0, -M_PI / 2);

      // Final orientation: target * 45deg_x * -90deg_z
      final_orientation_q = temp_orientation_q * q_neg_90_deg_z;
      final_orientation_q.normalize();
    }
    else
    {
      ROS_ERROR("Invalid object ID: %d", id);
      res.success = false;
      return true;
    }

    // Set the orientation for the above pose
    above_pose.orientation = tf2::toMsg(final_orientation_q);

    // Apply gripper offset of 0.05m (changed from -0.06)
    tf2::Vector3 gripper_offset(-0.05, 0.0, 0.0);
    tf2::Vector3 offset_world = tf2::quatRotate(final_orientation_q, gripper_offset);

    above_pose.position.x += offset_world.x();
    above_pose.position.y += offset_world.y();
    above_pose.position.z += offset_world.z();

    ROS_INFO("Step 1: Moving above object with adjusted orientation at x=%.3f, y=%.3f, z=%.3f",
             above_pose.position.x, above_pose.position.y, above_pose.position.z);

    arm_group->setPoseTarget(above_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm_group->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success1)
    {
      ROS_ERROR("Failed to plan movement above object with adjusted orientation!");
      res.success = false;
      return true;
    }

    moveit::core::MoveItErrorCode execute_result1 = arm_group->execute(plan1);
    if (execute_result1 != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to move above object!");
      res.success = false;
      return true;
    }

    ROS_INFO("Successfully moved above target!");

    // STEP 2: Cartesian move down
    ROS_INFO("STEP 2: Moving down to grasp position with Cartesian path");
    
    // Calculate the grasp position
    geometry_msgs::Pose grasp_pose = above_pose;  // Start with above pose (keeps orientation)
    
    // Move down to target Z position minus half object height, plus 0.05 for finger clearance
    grasp_pose.position.z = target_pose.position.z - (object_height / 2.0) + 0.05;
    
    ROS_INFO("Moving to grasp position at z=%.3f (target_z=%.3f, object_height=%.3f)",
             grasp_pose.position.z, target_pose.position.z, object_height);
    
    // Create waypoints for Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(above_pose);   // Start position
    waypoints.push_back(grasp_pose);   // End position
    
    // Plan Cartesian path
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // Disable jump threshold
    const double eef_step = 0.01;       // 1cm interpolation steps
    double fraction = arm_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("Cartesian path planned (%.2f%% achieved)", fraction * 100.0);
    
    if (fraction < 0.95)  // If less than 95% of path achieved
    {
      ROS_ERROR("Could not plan full Cartesian path down to object!");
      res.success = false;
      return true;
    }
    
    // Execute the Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    plan2.trajectory_ = trajectory;
    
    moveit::core::MoveItErrorCode execute_result2 = arm_group->execute(plan2);
    if (execute_result2 != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to execute Cartesian move down!");
      res.success = false;
      return true;
    }
    
    ROS_INFO("Successfully moved to grasp position!");

    // STEP 3: Close the gripper
    ROS_INFO("STEP 3: Closing gripper for object ID %d", id);
    
    try {
      // Close gripper with appropriate width for the object
      gripper.close(id);
      
      // Small delay to ensure gripper has closed
      ros::Duration(0.5).sleep();
      
      ROS_INFO("Gripper closed successfully!");
    }
    catch (const std::exception& e) {
      ROS_ERROR("Failed to close gripper: %s", e.what());
      res.success = false;
      return true;
    }

    // STEP 4: Lift object straight up
    ROS_INFO("STEP 4: Lifting object up by 0.6m");
    
    // Get current pose (should be at grasp position)
    geometry_msgs::Pose lift_pose = grasp_pose;
    
    // Move up 0.6m in Z direction
    lift_pose.position.z += 0.6;
    
    ROS_INFO("Lifting to position at z=%.3f", lift_pose.position.z);
    
    // Create waypoints for Cartesian lift
    std::vector<geometry_msgs::Pose> lift_waypoints;
    lift_waypoints.push_back(grasp_pose);  // Start position
    lift_waypoints.push_back(lift_pose);   // End position (0.6m up)
    
    // Plan Cartesian path for lifting
    moveit_msgs::RobotTrajectory lift_trajectory;
    const double lift_jump_threshold = 0.0;  // Disable jump threshold
    const double lift_eef_step = 0.01;       // 1cm interpolation steps
    double lift_fraction = arm_group->computeCartesianPath(lift_waypoints, lift_eef_step, 
                                                           lift_jump_threshold, lift_trajectory);
    
    ROS_INFO("Lift Cartesian path planned (%.2f%% achieved)", lift_fraction * 100.0);
    
    if (lift_fraction < 0.95)  // If less than 95% of path achieved
    {
      ROS_ERROR("Could not plan full Cartesian path for lifting!");
      res.success = false;
      return true;
    }
    
    // Execute the lift
    moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
    lift_plan.trajectory_ = lift_trajectory;
    
    moveit::core::MoveItErrorCode lift_result = arm_group->execute(lift_plan);
    if (lift_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to execute lift!");
      res.success = false;
      return true;
    }
    
    ROS_INFO("Successfully lifted object!");

    ROS_INFO("Pick sequence completed successfully!");
    res.success = true;
  }
  catch (const std::exception &e)
  {
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

  // Initialize TF Buffer and Listener and assign to global pointers
  tf2_ros::Buffer tfBuffer_obj;
  tf2_ros::TransformListener tfListener_obj(tfBuffer_obj);
  tfBuffer = &tfBuffer_obj;
  tfListener = &tfListener_obj; // Assigning address to global pointer

  // MoveIt interfaces with error handling and debug info
  try
  {
    // Debug: Print available planning groups
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    const std::vector<std::string> &group_names = kinematic_model->getJointModelGroupNames();

    ROS_INFO("Available planning groups:");
    for (const std::string &group_name : group_names)
    {
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
  }
  catch (const std::exception &e)
  {
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
  // Don't delete tfListener since it was not allocated with 'new'
  return 0;
}