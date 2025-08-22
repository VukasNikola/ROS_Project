/**
 * @file node_C.cpp
 * @brief MoveIt manipulation and collision object management node
 * 
 * This node handles:
 * - Real-time collision object management with object freezing during manipulation
 * - Pick and place operations for various object types (cylinders, cubes, triangular prisms)
 * - Coordination between MoveIt planning scene and Gazebo physics simulation
 * - Gripper control and object attachment/detachment
 * 
 * Object Management Strategy:
 * - Objects are frozen (pose updates disabled) during manipulation to prevent planning conflicts
 * - Smart update logic reduces unnecessary collision scene updates for better performance
 * - Supports both primitive shapes (cylinders, boxes) and mesh objects (triangular prisms)
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include "assignment2_package/PickObject.h"
#include "assignment2_package/ObjectPose.h"
#include "assignment2_package/ObjectPoseArray.h"
#include "assignment2_package/PlaceObject.h"
#include "assignment2_package/gripper_control.h"
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <boost/variant.hpp>
#include <map>
#include <cmath>
#include <set>
#include <cstdint>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>
#include <geometry_msgs/TransformStamped.h>

// =============================================================================
// CONSTANTS AND CONFIGURATION
// =============================================================================

// MoveIt planning groups
static const std::string PLANNING_GROUP_ARM = "arm_torso";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

// Reference frames and links
static const std::string BASE_FRAME = "base_footprint";
static const std::string GRIPPER_LINK_NAME = "arm_7_link";        // For Gazebo attachment
static const std::string MOVEIT_ATTACH_LINK = "gripper_grasping_frame"; // For MoveIt attachment

// Safe arm position for camera visibility and travel
static const std::map<std::string, double> SAFE_TRAVEL_JOINTS = {
    {"torso_lift_joint", 0.200},
    {"arm_1_joint", 0.200},
    {"arm_2_joint", -1.339},
    {"arm_3_joint", -0.200},
    {"arm_4_joint", 1.938},
    {"arm_5_joint", -1.570},
    {"arm_6_joint", 1.370},
    {"arm_7_joint", 0.00}};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// MoveIt interfaces
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
ros::NodeHandle *nh_ptr;

// Object tracking and state management
std::map<int, geometry_msgs::PoseStamped> detected_tags;  // Currently detected objects
std::map<int, geometry_msgs::PoseStamped> frozen_objects; // Frozen object poses (unused but kept for future extension)

// Object manipulation state
int currently_manipulated_id = -1; // ID of object being manipulated (-1 = none)
std::string attached_object_id;    // MoveIt collision object ID of attached object

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * @brief Move arm to safe position for camera visibility and travel
 * @return true if successful, false otherwise
 */
bool moveArmToSafePosition()
{
  ROS_INFO("Moving arm to safe position");
  try
  {
    moveit::planning_interface::MoveGroupInterface arm_torso_group("arm_torso");

    // Configure movement parameters for safety
    arm_torso_group.setGoalPositionTolerance(0.03);
    arm_torso_group.setGoalOrientationTolerance(0.14);
    arm_torso_group.setMaxAccelerationScalingFactor(0.5);
    arm_torso_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_group.setPlanningTime(15.0);
    arm_torso_group.setNumPlanningAttempts(10);

    arm_torso_group.setJointValueTarget(SAFE_TRAVEL_JOINTS);

    // Plan and execute with safety checks
    moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
    bool plan_success = (arm_torso_group.plan(safe_plan) ==
                         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (plan_success)
    {
      // Execute with reduced speed for safety
      arm_torso_group.setMaxVelocityScalingFactor(0.3);
      arm_torso_group.setMaxAccelerationScalingFactor(0.3);

      moveit::core::MoveItErrorCode exec_result = arm_torso_group.execute(safe_plan);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ros::Duration(1.0).sleep();
        return true;
      }
      else
      {
        ROS_WARN("Failed to execute safe position movement");
        return false;
      }
    }
    else
    {
      ROS_WARN("Failed to plan safe position movement");
      return false;
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN("Exception moving arm to safe position: %s", e.what());
    return false;
  }
}

/**
 * @brief Broadcast placement pose as TF frame for visualization
 * @param pose The pose to broadcast
 * @param child_frame The frame name to use
 */
void broadcastPlacementPose(const geometry_msgs::PoseStamped &pose, const std::string &child_frame)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped tf_msg;

  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = pose.header.frame_id;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = pose.pose.position.x;
  tf_msg.transform.translation.y = pose.pose.position.y;
  tf_msg.transform.translation.z = pose.pose.position.z;
  tf_msg.transform.rotation = pose.pose.orientation;

  br.sendTransform(tf_msg);
}

/**
 * @brief Generate collision object ID for a given tag
 * @param tag_id The tag ID
 * @return Unique collision object identifier
 */
std::string getCollisionObjectId(int tag_id)
{
  if (tag_id >= 7 && tag_id <= 9)
  {
    return "tag_mesh_" + std::to_string(tag_id);
  }
  else
  {
    return "tag_object_" + std::to_string(tag_id);
  }
}

/**
 * @brief Add or update a primitive collision object in the planning scene
 * @param id Unique object identifier
 * @param primitive The primitive shape definition
 * @param pose Object pose in the specified frame
 * @param frame_id Reference frame for the pose
 */
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

/**
 * @brief Add or update a mesh collision object in the planning scene
 * @param id Unique object identifier
 * @param mesh_resource Path to the mesh file
 * @param pose Object pose in the specified frame
 * @param frame_id Reference frame for the pose
 * @param scale Scale factors for X, Y, Z axes
 */
void addMeshCollisionObject(const std::string &id,
                            const std::string &mesh_resource,
                            const geometry_msgs::Pose &pose,
                            const std::string &frame_id,
                            const Eigen::Vector3d &scale)
{
  moveit_msgs::CollisionObject co;
  co.id = id;
  co.header.frame_id = frame_id;

  // Load and scale the mesh
  shapes::Mesh *m = shapes::createMeshFromResource(mesh_resource, scale);

  // Convert to ROS message format
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  const shape_msgs::Mesh &mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Add to collision object and apply
  co.meshes.push_back(mesh_msg);
  co.mesh_poses.push_back(pose);
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface->applyCollisionObjects({co});
}

// =============================================================================
// OBJECT POSE MANAGEMENT
// =============================================================================

/**
 * @brief Callback for object pose updates with intelligent freezing logic
 * 
 * This callback implements smart object management:
 * - Objects being manipulated are frozen (no pose updates)
 * - Smart update logic reduces unnecessary collision scene updates
 * - Automatically unfreezes objects when manipulation ends
 * - Handles both primitive shapes and mesh objects
 * 
 * @param msg Array of detected object poses
 */
void objectPosesCallback(const assignment2_package::ObjectPoseArray::ConstPtr &msg)
{
  std::set<int> seen_ids;
  static std::map<int, ros::Time> last_object_update;

  // Precompute mesh resources for triangular prisms
  std::string pkg_path = ros::package::getPath("tiago_iaslab_simulation");
  std::string mesh_uri = "file://" + pkg_path + "/meshes/triangle_centered.stl";
  Eigen::Vector3d prism_scale(0.5, 0.5, 0.5);

  // Process each detected object
  for (const auto &entry : msg->objects)
  {
    int tag_id = entry.id;

    // Skip reference tag (tag 10)
    if (tag_id == 10)
    {
      continue;
    }

    seen_ids.insert(tag_id);

    // CORE FREEZING LOGIC: Skip updates for objects being manipulated
    if (currently_manipulated_id == tag_id)
    {
      ROS_DEBUG("Object %d frozen during manipulation", tag_id);
      continue;
    }

    // Unfreeze objects that are no longer being manipulated
    if (frozen_objects.find(tag_id) != frozen_objects.end())
    {
      frozen_objects.erase(tag_id);
      ROS_DEBUG("Object %d unfrozen", tag_id);
    }

    geometry_msgs::Pose obj_pose = entry.pose.pose;
    std::string obj_id = getCollisionObjectId(tag_id);

    // Smart update logic: only update when necessary
    bool should_update = false;
    ros::Time now = ros::Time::now();

    if (detected_tags.find(tag_id) != detected_tags.end())
    {
      // Check if object moved significantly or needs periodic update
      geometry_msgs::Pose old_pose = detected_tags[tag_id].pose;
      double distance = sqrt(pow(entry.pose.pose.position.x - old_pose.position.x, 2) +
                             pow(entry.pose.pose.position.y - old_pose.position.y, 2) +
                             pow(entry.pose.pose.position.z - old_pose.position.z, 2));

      ros::Time last_update = last_object_update[tag_id];

      if (distance > 0.003) // 3mm movement threshold
      {
        should_update = true;
      }
      else if ((now - last_update).toSec() > 2.0) // Force update every 2 seconds
      {
        should_update = true;
      }
      else
      {
        // Minor change - just update stored pose without collision scene update
        detected_tags[tag_id] = entry.pose;
      }
    }
    else
    {
      // First detection - always add
      should_update = true;
    }

    // Update collision scene only when necessary
    if (should_update)
    {
      if (tag_id >= 7 && tag_id <= 9)
      {
        // Handle triangular prism mesh objects
        tf2::Quaternion tag_q;
        tf2::fromMsg(entry.pose.pose.orientation, tag_q);
        tf2::Quaternion corr_q;
        corr_q.setRPY(M_PI, 0.0, M_PI / 2.0);
        tf2::Quaternion final_q = tag_q * corr_q;
        obj_pose.orientation = tf2::toMsg(final_q);

        // Remove existing before updating (mesh objects need longer processing time)
        if (detected_tags.find(tag_id) != detected_tags.end())
        {
          std::vector<std::string> to_remove = {obj_id};
          planning_scene_interface->removeCollisionObjects(to_remove);
          ros::Duration(0.2).sleep();
        }

        addMeshCollisionObject(obj_id, mesh_uri, obj_pose, BASE_FRAME, prism_scale);
      }
      else
      {
        // Handle primitive objects (cylinders and cubes)
        shape_msgs::SolidPrimitive primitive;
        double height_offset = 0.0;

        if (tag_id >= 1 && tag_id <= 3)
        {
          // Cylinder objects
          primitive.type = primitive.CYLINDER;
          primitive.dimensions = {0.1, 0.03}; // height, radius
          height_offset = primitive.dimensions[0] / 2.0;
        }
        else if (tag_id >= 4 && tag_id <= 6)
        {
          // Box objects
          primitive.type = primitive.BOX;
          primitive.dimensions = {0.05, 0.05, 0.05};
          height_offset = primitive.dimensions[2] / 2.0;
        }
        else
        {
          ROS_WARN("Unknown tag ID %d - skipping", tag_id);
          continue;
        }

        // Adjust Z position for collision object center
        obj_pose.position.z -= (height_offset - 0.005);

        // Remove existing before updating
        if (detected_tags.find(tag_id) != detected_tags.end())
        {
          std::vector<std::string> to_remove = {obj_id};
          planning_scene_interface->removeCollisionObjects(to_remove);
          ros::Duration(0.1).sleep();
        }

        addCollisionObject(obj_id, primitive, obj_pose, BASE_FRAME);
      }

      detected_tags[tag_id] = entry.pose;
      last_object_update[tag_id] = now;
    }
  }

  // Clean up objects that are no longer detected
  std::vector<std::string> to_remove;
  for (auto it = detected_tags.begin(); it != detected_tags.end();)
  {
    int id = it->first;

    if (!seen_ids.count(id))
    {
      // Don't remove objects currently being manipulated
      if (currently_manipulated_id == id)
      {
        ++it;
        continue;
      }

      // Remove from scene and cleanup
      std::string obj_id = getCollisionObjectId(id);
      to_remove.push_back(obj_id);
      last_object_update.erase(id);
      it = detected_tags.erase(it);
    }
    else
    {
      ++it;
    }
  }

  if (!to_remove.empty())
  {
    planning_scene_interface->removeCollisionObjects(to_remove);
  }
}

// =============================================================================
// SERVICE CALLBACKS
// =============================================================================

/**
 * @brief Service callback for picking objects
 * 
 * Implements a complete pick sequence:
 * 1. Freeze object updates to prevent planning conflicts
 * 2. Open gripper and move to approach position
 * 3. Move down to grasp position using Cartesian or regular planning
 * 4. Attach object in both Gazebo physics and MoveIt planning scene
 * 5. Close gripper and lift object
 * 6. Move to safe travel position
 * 
 * @param req Pick request containing target object ID and pose
 * @param res Pick response indicating success/failure
 * @return true (service always returns, check res.success for actual result)
 */
bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  ROS_INFO("=== PICK OBJECT SERVICE: ID %d ===", req.target_id);

  int id = req.target_id;

  // STEP 1: Freeze object updates for this object
  currently_manipulated_id = id;

  try
  {
    // Initialize gripper controller
    gripper_control::GripperController gripper("gripper");
    gripper.registerNamedTarget("open", 0.044);
    gripper.registerNamedTarget("closed", 0.0);

    // Open gripper and configure arm
    gripper.open();
    arm_group->setEndEffectorLink("gripper_grasping_frame");

    // Get target pose and calculate approach strategy
    geometry_msgs::Pose target_pose = req.target.pose;
    attached_object_id = getCollisionObjectId(id);

    geometry_msgs::Pose above_pose;
    tf2::Quaternion final_orientation_q;
    double object_height;

    // Object-specific approach calculation
    if (id >= 1 && id <= 3)
    {
      // Cylinders: top-down approach
      object_height = 0.1;
      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, 0);
      final_orientation_q = downward_q;

      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 4 && id <= 6)
    {
      // Cubes: top-down with yaw preservation
      object_height = 0.0495;
      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(target_q).getRPY(roll, pitch, yaw);

      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, yaw);
      final_orientation_q = downward_q;

      above_pose.position = target_pose.position;
      above_pose.position.z += 0.35;
    }
    else if (id >= 7 && id <= 9)
    {
      // Triangular prisms: angled approach
      object_height = 0.032;
      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);

      tf2::Quaternion q_rx45, q_rx180, q_rz90;
      q_rx45.setRPY(M_PI / 4, 0, 0);
      q_rx180.setRPY(M_PI, 0, 0);
      q_rz90.setRPY(0, 0, M_PI / 2);

      final_orientation_q = target_q * q_rx45 * q_rx180 * q_rz90;
      final_orientation_q.normalize();

      tf2::Vector3 approach_offset(0, 0, -0.35);
      tf2::Vector3 offset_world = tf2::quatRotate(final_orientation_q, approach_offset);

      above_pose.position = target_pose.position;
      above_pose.position.x += offset_world.x();
      above_pose.position.y += offset_world.y();
      above_pose.position.z += offset_world.z();
    }
    else
    {
      ROS_ERROR("Invalid object ID: %d", id);
      currently_manipulated_id = -1;
      res.success = false;
      return true;
    }

    // Apply gripper offset and set final orientation
    above_pose.orientation = tf2::toMsg(final_orientation_q);
    tf2::Vector3 gripper_offset(-0.05, 0.0, 0.0);
    tf2::Vector3 offset_world = tf2::quatRotate(final_orientation_q, gripper_offset);
    above_pose.position.x += offset_world.x();
    above_pose.position.y += offset_world.y();
    above_pose.position.z += offset_world.z();

    // STEP 2: Move above object
    ROS_INFO("Moving above object");
    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setGoalPositionTolerance(0.010);
    arm_group->setGoalOrientationTolerance(0.03);
    arm_group->setPlanningTime(10.0);
    arm_group->setNumPlanningAttempts(10);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    arm_group->setPoseTarget(above_pose);
    arm_group->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (arm_group->plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Initial planning failed, trying relaxed tolerances");
      arm_group->setGoalPositionTolerance(0.015);
      arm_group->setGoalOrientationTolerance(0.015);
      arm_group->setPlanningTime(15.0);
      arm_group->setNumPlanningAttempts(20);

      if (arm_group->plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR("Failed to plan movement above object");
        currently_manipulated_id = -1;
        res.success = false;
        return true;
      }
    }

    if (arm_group->execute(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to move above object");
      currently_manipulated_id = -1;
      res.success = false;
      return true;
    }

    arm_group->clearPoseTargets();
    ros::Duration(0.5).sleep();

    // STEP 3: Move down to grasp position
    ROS_INFO("Moving to grasp position");
    geometry_msgs::Pose grasp_pose = above_pose;
    grasp_pose.position.z = target_pose.position.z - (object_height / 2.0) + 0.025;

    bool grasp_motion_success = false;

    // Try Cartesian path first for smooth motion
    std::vector<geometry_msgs::Pose> waypoints = {above_pose, grasp_pose};
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.015;
    const bool avoid_collisions = true;
    arm_group->setStartStateToCurrentState();

    moveit_msgs::MoveItErrorCodes cart_err;
    double fraction = arm_group->computeCartesianPath(
        waypoints, eef_step, trajectory, avoid_collisions, &cart_err);

    if (fraction >= 0.85)
    {
      // Time-parameterize for smoother motion
      robot_trajectory::RobotTrajectory rt(
          arm_group->getCurrentState()->getRobotModel(), arm_group->getName());
      rt.setRobotTrajectoryMsg(*arm_group->getCurrentState(), trajectory);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, 0.3, 0.3);
      rt.getRobotTrajectoryMsg(trajectory);

      moveit::planning_interface::MoveGroupInterface::Plan plan2;
      plan2.trajectory_ = trajectory;

      if (arm_group->execute(plan2) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        grasp_motion_success = true;
      }
    }

    // Fallback to regular planning if Cartesian failed
    if (!grasp_motion_success)
    {
      arm_group->setPoseTarget(grasp_pose);
      arm_group->setStartStateToCurrentState();
      arm_group->setMaxVelocityScalingFactor(0.3);
      arm_group->setMaxAccelerationScalingFactor(0.3);

      moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
      if (arm_group->plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        if (arm_group->execute(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          grasp_motion_success = true;
        }
      }
    }

    if (!grasp_motion_success)
    {
      ROS_ERROR("Failed to reach grasp position");
      currently_manipulated_id = -1;
      res.success = false;
      return true;
    }

    arm_group->clearPoseTargets();
    ros::Duration(1.0).sleep();

    // STEP 4: Attach object in Gazebo physics
    ROS_INFO("Attaching object in Gazebo");
    std::string gazebo_attach_service = "/link_attacher_node/attach";
    if (ros::service::waitForService(gazebo_attach_service, ros::Duration(2.0)))
    {
      ros::ServiceClient attach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(gazebo_attach_service);
      gazebo_ros_link_attacher::Attach attach_srv;
      attach_srv.request.model_name_1 = "tiago";
      attach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      // Map object ID to Gazebo model names
      std::map<int, std::pair<std::string, std::string>> gazebo_objects = {
          {1, {"Hexagon", "Hexagon_link"}},
          {2, {"Hexagon_2", "Hexagon_2_link"}},
          {3, {"Hexagon_3", "Hexagon_3_link"}},
          {4, {"cube", "cube_link"}},
          {5, {"cube_5", "cube_5_link"}},
          {6, {"cube_6", "cube_6_link"}},
          {7, {"Triangle", "Triangle_link"}},
          {8, {"Triangle_8", "Triangle_8_link"}},
          {9, {"Triangle_9", "Triangle_9_link"}}
      };

      if (gazebo_objects.find(id) != gazebo_objects.end())
      {
        attach_srv.request.model_name_2 = gazebo_objects[id].first;
        attach_srv.request.link_name_2 = gazebo_objects[id].second;

        if (attach_client.call(attach_srv) && attach_srv.response.ok)
        {
          ROS_INFO("Object attached in Gazebo");
        }
        else
        {
          ROS_WARN("Failed to attach in Gazebo - continuing");
        }
      }
    }

    // STEP 5: Attach in MoveIt planning scene
    ROS_INFO("Attaching object in MoveIt");
    moveit_msgs::AttachedCollisionObject attached_co;
    attached_co.link_name = MOVEIT_ATTACH_LINK;
    attached_co.object.header.frame_id = MOVEIT_ATTACH_LINK;
    attached_co.object.id = attached_object_id;

    geometry_msgs::Pose attach_pose;
    attach_pose.orientation.w = 1.0;
    attach_pose.position.x = -0.05; // Gripper offset
    attach_pose.position.y = 0.0;
    attach_pose.position.z = 0.0;

    attached_co.object.primitive_poses.push_back(attach_pose);
    attached_co.object.mesh_poses.push_back(attach_pose);

    // Define touch links to avoid self-collision
    attached_co.touch_links = {
        "gripper_left_finger_link", "gripper_right_finger_link", "gripper_grasping_frame",
        "gripper_grasping_frame_Y", "gripper_grasping_frame_Z",
        "arm_7_link", "arm_6_link", "arm_5_link", "arm_4_link", "arm_3_link"};

    attached_co.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface->applyAttachedCollisionObject(attached_co);

    // Remove standalone collision object
    std::vector<std::string> to_remove_after_attach = {attached_object_id};
    planning_scene_interface->removeCollisionObjects(to_remove_after_attach);
    ros::Duration(0.5).sleep();

    // STEP 6: Close gripper
    ROS_INFO("Closing gripper");
    gripper.close(id);
    ros::Duration(0.5).sleep();

    // STEP 7: Lift object
    ROS_INFO("Lifting object");
    geometry_msgs::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.3;

    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setGoalPositionTolerance(0.015);
    arm_group->setGoalOrientationTolerance(0.015);
    arm_group->setPlanningTime(15.0);
    arm_group->setNumPlanningAttempts(20);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    arm_group->setPoseTarget(lift_pose);
    arm_group->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
    if (arm_group->plan(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      if (arm_group->execute(lift_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR("Failed to lift object");
        res.success = false;
        return true;
      }
    }
    else
    {
      ROS_ERROR("Failed to plan lift movement");
      res.success = false;
      return true;
    }

    ros::Duration(1.0).sleep();

    // STEP 8: Move to safe position
    ROS_INFO("Moving to safe travel position");
    moveArmToSafePosition();

    ROS_INFO("Pick sequence completed successfully");
    res.success = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in pick operation: %s", e.what());
    currently_manipulated_id = -1;
    res.success = false;
  }

  return true;
}

/**
 * @brief Service callback for placing objects
 * 
 * Implements a complete place sequence:
 * 1. Calculate placement pose with object-specific Z offset
 * 2. Move arm to placement position
 * 3. Detach object from MoveIt planning scene
 * 4. Detach object from Gazebo physics
 * 5. Open gripper and retreat
 * 6. Move to safe position and unfreeze object
 * 
 * @param req Place request containing target object ID and placement pose
 * @param res Place response indicating success/failure
 * @return true (service always returns, check res.success for actual result)
 */
bool placeObjectCallback(assignment2_package::PlaceObject::Request &req,
                         assignment2_package::PlaceObject::Response &res)
{
  ROS_INFO("=== PLACE OBJECT SERVICE: ID %d ===", req.target_id);

  try
  {
    int id = req.target_id;

    // STEP 1: Calculate placement pose with object-specific offset
    double z_offset;
    if (id >= 1 && id <= 3)
    {
      z_offset = 0.113; // Cylinders
    }
    else if (id >= 4 && id <= 9)
    {
      z_offset = 0.06; // Cubes and prisms
    }
    else
    {
      ROS_ERROR("Invalid object ID: %d", id);
      res.success = false;
      return true;
    }

    geometry_msgs::PoseStamped placement_pose = req.target_pose;
    double table_surface_z = placement_pose.pose.position.z;
    placement_pose.pose.position.z = table_surface_z + z_offset;

    // Broadcast for visualization
    broadcastPlacementPose(placement_pose, "placement_pose_frame");

    ROS_INFO("Placing at height: %.3fm (table: %.3fm + offset: %.2fm)",
             placement_pose.pose.position.z, table_surface_z, z_offset);

    // STEP 2: Move to placement position
    ROS_INFO("Moving to placement position");
    moveit::planning_interface::MoveGroupInterface arm_torso_group("arm_torso");
    arm_torso_group.setEndEffectorLink("gripper_grasping_frame");
    arm_torso_group.setPoseTarget(placement_pose.pose, "gripper_grasping_frame");

    // Configure planning parameters
    arm_torso_group.setPlanningTime(30.0);
    arm_torso_group.setNumPlanningAttempts(20);
    arm_torso_group.setGoalPositionTolerance(0.05);
    arm_torso_group.setGoalOrientationTolerance(0.2);
    arm_torso_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_group.setMaxAccelerationScalingFactor(0.5);

    moveit::planning_interface::MoveGroupInterface::Plan placement_plan;
    bool success = (arm_torso_group.plan(placement_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      // Try with alternative end effector
      arm_torso_group.setEndEffectorLink("arm_tool_link");
      arm_torso_group.setPoseTarget(placement_pose.pose, "arm_tool_link");
      success = (arm_torso_group.plan(placement_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!success)
      {
        ROS_ERROR("Failed to plan placement movement");
        res.success = false;
        return true;
      }
    }

    moveit::core::MoveItErrorCode exec_result = arm_torso_group.execute(placement_plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Placement execution warning (code %d), continuing", exec_result.val);
    }

    ros::Duration(2.0).sleep();

    // STEP 3: Detach from MoveIt planning scene
    ROS_INFO("Detaching from MoveIt planning scene");
    moveit_msgs::AttachedCollisionObject detach_from_moveit;
    detach_from_moveit.link_name = MOVEIT_ATTACH_LINK;
    detach_from_moveit.object.id = attached_object_id;
    detach_from_moveit.object.operation = moveit_msgs::CollisionObject::REMOVE;
    detach_from_moveit.object.header.frame_id = MOVEIT_ATTACH_LINK;
    planning_scene_interface->applyAttachedCollisionObject(detach_from_moveit);
    ros::Duration(0.5).sleep();

    // Temporarily remove object to avoid gripper collision issues
    if (!attached_object_id.empty())
    {
      std::vector<std::string> rm_ids = {attached_object_id};
      planning_scene_interface->removeCollisionObjects(rm_ids);
      ros::Duration(0.2).sleep();
      detected_tags.erase(id);
    }

    // STEP 4: Detach from Gazebo physics
    ROS_INFO("Detaching from Gazebo physics");
    std::string gazebo_detach_service = "/link_attacher_node/detach";
    if (ros::service::waitForService(gazebo_detach_service, ros::Duration(2.0)))
    {
      ros::ServiceClient detach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(gazebo_detach_service);
      gazebo_ros_link_attacher::Attach detach_srv;
      detach_srv.request.model_name_1 = "tiago";
      detach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      // Map object ID to Gazebo model names (same as in pick)
      std::map<int, std::pair<std::string, std::string>> gazebo_objects = {
          {1, {"Hexagon", "Hexagon_link"}},
          {2, {"Hexagon_2", "Hexagon_2_link"}},
          {3, {"Hexagon_3", "Hexagon_3_link"}},
          {4, {"cube", "cube_link"}},
          {5, {"cube_5", "cube_5_link"}},
          {6, {"cube_6", "cube_6_link"}},
          {7, {"Triangle", "Triangle_link"}},
          {8, {"Triangle_8", "Triangle_8_link"}},
          {9, {"Triangle_9", "Triangle_9_link"}}
      };

      if (gazebo_objects.find(id) != gazebo_objects.end())
      {
        detach_srv.request.model_name_2 = gazebo_objects[id].first;
        detach_srv.request.link_name_2 = gazebo_objects[id].second;

        if (detach_client.call(detach_srv) && detach_srv.response.ok)
        {
          ROS_INFO("Object detached from Gazebo");
        }
        else
        {
          ROS_WARN("Failed to detach from Gazebo");
        }
      }
    }

    ros::Duration(1.0).sleep(); // Allow physics to settle

    // STEP 5: Open gripper
    ROS_INFO("Opening gripper");
    gripper_control::GripperController gripper("gripper");
    gripper.registerNamedTarget("open", 0.044);
    gripper.open();
    ros::Duration(0.5).sleep();

    // STEP 6: Small retreat
    ROS_INFO("Retreating from placement");
    geometry_msgs::PoseStamped current_pose_stamped = arm_group->getCurrentPose();
    geometry_msgs::Pose retreat_pose = current_pose_stamped.pose;
    retreat_pose.position.z += 0.1;

    arm_group->setPoseTarget(retreat_pose);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.3);

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    if (arm_group->plan(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      arm_group->execute(retreat_plan);
    }

    arm_group->setMaxVelocityScalingFactor(0.9);
    arm_group->setMaxAccelerationScalingFactor(0.9);

    // STEP 7: Move to safe position
    ROS_INFO("Moving to safe position");
    moveArmToSafePosition();

    // STEP 8: Unfreeze object for future collision detection
    ROS_INFO("Unfreezing object %d for collision detection", id);
    currently_manipulated_id = -1;
    frozen_objects.erase(id);
    attached_object_id = "";

    ROS_INFO("Place sequence completed successfully");
    res.success = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in place operation: %s", e.what());
    currently_manipulated_id = -1;
    attached_object_id = "";
    res.success = false;
  }

  return true;
}

// =============================================================================
// MAIN FUNCTION
// =============================================================================

/**
 * @brief Main function - initializes the node and starts spinning
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_C");
  ros::NodeHandle nh;
  nh_ptr = &nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize MoveIt interfaces
  try
  {
    ROS_INFO("Initializing MoveIt interfaces...");

    // Load robot model and display available planning groups
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    const std::vector<std::string> &group_names = kinematic_model->getJointModelGroupNames();

    ROS_INFO("Available planning groups:");
    for (const std::string &group_name : group_names)
    {
      ROS_INFO("  - %s", group_name.c_str());
    }

    // Initialize MoveIt interfaces
    static moveit::planning_interface::MoveGroupInterface arm_mg(PLANNING_GROUP_ARM);
    static moveit::planning_interface::PlanningSceneInterface psi;

    arm_group = &arm_mg;
    planning_scene_interface = &psi;

    // Configure default planning parameters
    arm_group->setPlanningTime(10.0);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    ROS_INFO("MoveIt interfaces initialized successfully");
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Failed to initialize MoveIt: %s", e.what());
    return -1;
  }

  // Setup ROS communication
  ros::Subscriber object_sub = nh.subscribe("object_poses", 1, objectPosesCallback);
  ros::ServiceServer pick_service = nh.advertiseService("pick_object", pickObjectCallback);
  ros::ServiceServer place_service = nh.advertiseService("place_object", placeObjectCallback);

  ROS_INFO("Node C ready - MoveIt manipulation controller with object freezing");
  ros::waitForShutdown();

  return 0;
}