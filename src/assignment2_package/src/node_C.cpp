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


// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm_torso";   // Changed from "arm" to "arm_torso"
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
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  ROS_INFO("PickObject service called - executing pick sequence");
  try {
    // Set the end effector link
    arm_group->setEndEffectorLink("gripper_grasping_frame");
    
    // Get object ID and determine height
    int id = req.target_id;
    double object_height;
    if (id >= 1 && id <= 3) {
      object_height = 0.1;  
    } else if (id >= 4 && id <= 6) {
      object_height = 0.05;  
    } else if (id >= 7 && id <= 9) {
      object_height = 0.035; 
    } else {
      ROS_ERROR("Invalid object ID: %d", id);
      res.success = false;
      return true;
    }
    
    // Get the target pose from the request
    geometry_msgs::Pose target_pose = req.target.pose;
    
    // STEP 1: Move above the object (20cm above)
    geometry_msgs::Pose above_pose = target_pose;
    above_pose.position.z += 0.2;  // Add 20cm to the z-axis
    
    ROS_INFO("Step 1: Moving above object at x=%.3f, y=%.3f, z=%.3f", 
             above_pose.position.x, above_pose.position.y, above_pose.position.z);
    
    arm_group->setPoseTarget(above_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm_group->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!success1) {
      ROS_ERROR("Failed to plan movement above object!");
      res.success = false;
      return true;
    }
    
    moveit::core::MoveItErrorCode execute_result1 = arm_group->execute(plan1);
    if (execute_result1 != moveit::core::MoveItErrorCode::SUCCESS) {
      ROS_ERROR("Failed to move above object!");
      res.success = false;
      return true;
    }
    
    ROS_INFO("Successfully moved above target!");
    
    // STEP 2: Straight line movement to grasp position
    ROS_INFO("Step 2: Moving in straight lines to grasp position");
    
    // Get current pose
    geometry_msgs::Pose current_pose = arm_group->getCurrentPose().pose;
    
    // SUBSTEP 2A: Move straight in -X direction of gripper frame by 0.11m
    std::vector<geometry_msgs::Pose> waypoints;
    
    // Transform -0.11m in gripper's X direction to world coordinates
    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::Vector3 move_forward(-0.11, 0.0, 0.0); // -11cm in gripper's X direction
    tf2::Vector3 forward_world = tf2::quatRotate(q_current, move_forward);
    
    geometry_msgs::Pose forward_pose = current_pose;
    forward_pose.position.x += forward_world.x();
    forward_pose.position.y += forward_world.y();
    forward_pose.position.z += forward_world.z();
    waypoints.push_back(forward_pose);
    
    ROS_INFO("Moving forward by 0.11m in gripper X direction");
    
    // Plan and execute Cartesian path for forward movement
    moveit_msgs::RobotTrajectory trajectory1;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // 1cm steps
    double fraction1 = arm_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
    
    if (fraction1 < 0.9) { // Less than 90% of path achievable
      ROS_ERROR("Could not compute forward Cartesian path (%.2f%% achieved)", fraction1 * 100.0);
      res.success = false;
      return true;
    }
    
    // Execute forward movement
    moveit::planning_interface::MoveGroupInterface::Plan forward_plan;
    forward_plan.trajectory_ = trajectory1;
    moveit::core::MoveItErrorCode forward_result = arm_group->execute(forward_plan);
    
    if (forward_result != moveit::core::MoveItErrorCode::SUCCESS) {
      ROS_ERROR("Failed to execute forward movement!");
      res.success = false;
      return true;
    }
    
    ROS_INFO("Successfully moved forward!");
    
    // SUBSTEP 2B: Move straight down to object level
    waypoints.clear();
    
    geometry_msgs::Pose current_pose_after_forward = arm_group->getCurrentPose().pose;
    geometry_msgs::Pose down_pose = current_pose_after_forward;
    
    // Move down to object Z minus half height (so gripper fingers are at object center)
    double target_z = target_pose.position.z - (object_height / 2.0);
    down_pose.position.z = target_z;
    waypoints.push_back(down_pose);
    
    ROS_INFO("Moving down to z=%.3f (object center level)", target_z);
    
    // Plan and execute Cartesian path for downward movement
    moveit_msgs::RobotTrajectory trajectory2;
    double fraction2 = arm_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory2);
    
    if (fraction2 < 0.9) { // Less than 90% of path achievable
      ROS_ERROR("Could not compute downward Cartesian path (%.2f%% achieved)", fraction2 * 100.0);
      res.success = false;
      return true;
    }
    
    // Execute downward movement
    moveit::planning_interface::MoveGroupInterface::Plan down_plan;
    down_plan.trajectory_ = trajectory2;
    moveit::core::MoveItErrorCode down_result = arm_group->execute(down_plan);
    
    if (down_result != moveit::core::MoveItErrorCode::SUCCESS) {
      ROS_ERROR("Failed to execute downward movement!");
      res.success = false;
      return true;
    }
    
    ROS_INFO("Successfully moved to grasp position!");
    
    // STEP 3: Close the gripper
    ROS_INFO("Step 3: Closing gripper...");
    
    // Create a separate move group for the gripper if it exists
    // Replace "gripper" with your actual gripper group name
    try {
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
      gripper_group.setNamedTarget("close");
      moveit::core::MoveItErrorCode gripper_result = gripper_group.move();
      
      if (gripper_result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Gripper closed successfully using gripper group!");
      } else {
        ROS_WARN("Gripper group move failed, trying joint control...");
        
        // Fallback: Direct joint control
        std::map<std::string, double> gripper_targets;
        gripper_targets["gripper_left_finger_joint"] = 0.01;
        gripper_targets["gripper_right_finger_joint"] = 0.01;
        
        gripper_group.setJointValueTarget(gripper_targets);
        gripper_group.move();
        ROS_INFO("Gripper closed using joint targets!");
      }
    } catch (const std::exception& e) {
      ROS_WARN("Could not use gripper group: %s", e.what());
      ROS_WARN("You may need to control the gripper through a separate action server or publisher");
      
      // Alternative: If you have a gripper action client or publisher, use it here
      // For example:
      // control_msgs::GripperCommandActionGoal gripper_goal;
      // gripper_goal.goal.command.position = 0.01; // closed position
      // gripper_action_client.sendGoal(gripper_goal);
    }
    
    ROS_INFO("Pick sequence completed successfully!");
    res.success = true;
    
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

  //  Broadcast static fingertip_frame
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped ts;
    ts.header.stamp    = ros::Time::now();
    ts.header.frame_id = "arm_tool_link";      // parent
    ts.child_frame_id  = "fingertip_frame";    // new frame
    ts.transform.translation.x = 0.21;
    ts.transform.translation.y =  0.0;
    ts.transform.translation.z =  0.0;
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = 0.0;
    ts.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(ts);
    ROS_INFO("Broadcasted static transform arm_tool_link → fingertip_frame (-0.21 m X)");
  }
  // Use AsyncSpinner to process service callbacks and MoveIt
  ros::AsyncSpinner spinner(2); // 2 threads should be sufficient
  spinner.start();

  // TF
  tfListener = new tf2_ros::TransformListener(tfBuffer);

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
  delete tfListener;
  return 0;
}