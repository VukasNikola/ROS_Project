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
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "assignment2_package/PickObject.h"
#include "assignment2_package/ObjectPose.h"
#include "assignment2_package/ObjectPoseArray.h"
#include "assignment2_package/PlaceObject.h"
#include "assignment2_package/GetObjectPose.h"
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

// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm_torso";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string BASE_FRAME = "base_footprint";
static const std::string GRIPPER_LINK_NAME = "arm_7_link";
// MoveIt will attach/detach using this link (keep Gazebo on GRIPPER_LINK_NAME)
static const std::string MOVEIT_ATTACH_LINK = "gripper_grasping_frame";

// Globals
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;
ros::ServiceClient get_obj_pose_client;
ros::NodeHandle *nh_ptr;

// Object tracking structures
std::map<int, geometry_msgs::PoseStamped> detected_tags;  // Currently detected objects
std::map<int, geometry_msgs::PoseStamped> frozen_objects; // Frozen object poses
std::map<int, bool> persistent_tables;                    // Track which tables have been created

// Object state management
int currently_manipulated_id = -1; // ID of object being manipulated (-1 = none)
std::string attached_object_id;    // MoveIt collision object ID of attached object
// Safe arm position for camera visibility
static const std::map<std::string, double> SAFE_TRAVEL_JOINTS = {
    {"torso_lift_joint", 0.200},
    {"arm_1_joint", 0.200},
    {"arm_2_joint", -1.339},
    {"arm_3_joint", -0.200},
    {"arm_4_joint", 1.938},
    {"arm_5_joint", -1.570},
    {"arm_6_joint", 1.370},
    {"arm_7_joint", 0.00}};
// Helper function to move arm to safe viewing position
bool moveArmToSafePosition()
{
  ROS_INFO("Moving arm to safe position for camera visibility");
  try
  {
    // Configure arm_torso for safe movement
    moveit::planning_interface::MoveGroupInterface arm_torso_group("arm_torso");

    // Set tolerances
    arm_torso_group.setGoalPositionTolerance(0.03);
    arm_torso_group.setGoalOrientationTolerance(0.14);
    arm_torso_group.setMaxAccelerationScalingFactor(0.5);
    arm_torso_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_group.setPlanningTime(15.0);
    arm_torso_group.setNumPlanningAttempts(10);

    arm_torso_group.setJointValueTarget(SAFE_TRAVEL_JOINTS);

    // Plan first, then execute
    moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
    bool plan_success = (arm_torso_group.plan(safe_plan) ==
                         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (plan_success)
    {
      // Slow down for safe movement
      arm_torso_group.setMaxVelocityScalingFactor(0.3);
      arm_torso_group.setMaxAccelerationScalingFactor(0.3);

      moveit::core::MoveItErrorCode exec_result = arm_torso_group.execute(safe_plan);

      // Reset scaling
      arm_torso_group.setMaxVelocityScalingFactor(0.5);
      arm_torso_group.setMaxAccelerationScalingFactor(0.5);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Arm moved to safe position successfully");
        ros::Duration(1.0).sleep();
        return true;
      }
      else
      {
        ROS_WARN("Failed to execute safe position movement (error %d)", exec_result.val);
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

void broadcastPlacementPose(const geometry_msgs::PoseStamped &pose, const std::string &child_frame)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped tf_msg;

  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = pose.header.frame_id; // "base_footprint"
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = pose.pose.position.x;
  tf_msg.transform.translation.y = pose.pose.position.y;
  tf_msg.transform.translation.z = pose.pose.position.z;
  tf_msg.transform.rotation = pose.pose.orientation;

  br.sendTransform(tf_msg);
}
// Helper: Get collision object ID for a given tag
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

// COMPLETELY REWRITTEN: Subscriber callback with proper freezing logic
void objectPosesCallback(const assignment2_package::ObjectPoseArray::ConstPtr &msg)
{
  std::set<int> seen_ids;

  // Static map to track when objects were last updated
  static std::map<int, ros::Time> last_object_update;

  // Precompute mesh resources
  std::string pkg_path = ros::package::getPath("tiago_iaslab_simulation");
  std::string mesh_uri = "file://" + pkg_path + "/meshes/triangle_centered.stl";
  Eigen::Vector3d prism_scale(0.5, 0.5, 0.5);

  for (const auto &entry : msg->objects)
  {
    int tag_id = entry.id;

    // Skip tag 10 (reference tag)
    if (tag_id == 10)
    {
      ROS_DEBUG("Skipping tag 10 (reference tag)");
      continue;
    }

    seen_ids.insert(tag_id);

    // CORE LOGIC: Check if this object is currently being manipulated
    if (currently_manipulated_id == tag_id)
    {
      // Object is frozen - don't update its collision object
      ROS_DEBUG("Object %d is currently being manipulated - keeping frozen pose", tag_id);
      continue;
    }

    // Check if this object was previously frozen and needs restoration
    if (frozen_objects.find(tag_id) != frozen_objects.end())
    {
      // Object was frozen but is no longer being manipulated
      // Remove it from frozen list and let it update normally
      frozen_objects.erase(tag_id);
      ROS_INFO("Object %d unfrozen - resuming normal pose updates", tag_id);
    }

    geometry_msgs::Pose obj_pose = entry.pose.pose;
    std::string obj_id = getCollisionObjectId(tag_id);

    // OPTION 3: Smart update logic for ALL objects
    bool should_update = false;
    ros::Time now = ros::Time::now();

    if (detected_tags.find(tag_id) != detected_tags.end())
    {
      // Object already exists - check if we should update it
      geometry_msgs::Pose old_pose = detected_tags[tag_id].pose;
      double distance = sqrt(pow(entry.pose.pose.position.x - old_pose.position.x, 2) +
                             pow(entry.pose.pose.position.y - old_pose.position.y, 2) +
                             pow(entry.pose.pose.position.z - old_pose.position.z, 2));

      ros::Time last_update = last_object_update[tag_id];

      if (distance > 0.003) // 3mm threshold for position changes
      {
        should_update = true;
        ROS_DEBUG("Object %d moved %.1fmm - updating collision object", tag_id, distance * 1000);
      }
      else if ((now - last_update).toSec() > 2.0) // Force update every 2 seconds
      {
        should_update = true;
        ROS_DEBUG("Object %d - forced periodic update (%.1fs since last)", tag_id, (now - last_update).toSec());
      }
      else
      {
        // Small change and recent update - just update stored pose
        detected_tags[tag_id] = entry.pose;
        ROS_DEBUG("Object %d - minor change, skipping collision update", tag_id);
      }
    }
    else
    {
      // First time seeing this object - always add
      should_update = true;
      ROS_INFO("Object %d - first detection, adding to collision scene", tag_id);
    }

    // Only update collision scene if needed
    if (should_update)
    {
      // Handle mesh objects (7-9)
      if (tag_id >= 7 && tag_id <= 9)
      {
        // Apply orientation corrections for mesh
        tf2::Quaternion tag_q;
        tf2::fromMsg(entry.pose.pose.orientation, tag_q);
        tf2::Quaternion corr_q;
        corr_q.setRPY(M_PI, 0.0, M_PI / 2.0);
        tf2::Quaternion final_q = tag_q * corr_q;
        obj_pose.orientation = tf2::toMsg(final_q);

        // Remove existing before updating
        if (detected_tags.find(tag_id) != detected_tags.end())
        {
          std::vector<std::string> to_remove = {obj_id};
          planning_scene_interface->removeCollisionObjects(to_remove);
          ros::Duration(0.2).sleep(); // Longer wait for mesh processing
        }

        addMeshCollisionObject(obj_id, mesh_uri, obj_pose, BASE_FRAME, prism_scale);
        detected_tags[tag_id] = entry.pose;
        last_object_update[tag_id] = now;
      }
      else
      {
        // Handle primitive objects (1-6)
        shape_msgs::SolidPrimitive primitive;
        double height_offset = 0.0;

        if (tag_id >= 1 && tag_id <= 3)
        {
          // Cylinders
          primitive.type = primitive.CYLINDER;
          primitive.dimensions = {0.1, 0.03}; // height, radius
          height_offset = primitive.dimensions[0] / 2.0;
        }
        else if (tag_id >= 4 && tag_id <= 6)
        {
          // Boxes
          primitive.type = primitive.BOX;
          primitive.dimensions = {0.05, 0.05, 0.05};
          height_offset = primitive.dimensions[2] / 2.0;
        }
        else
        {
          ROS_WARN("Unknown tag ID %d - skipping", tag_id);
          continue;
        }

        // Adjust Z position for collision object
        obj_pose.position.z -= (height_offset - 0.005);

        // Remove existing before updating
        if (detected_tags.find(tag_id) != detected_tags.end())
        {
          std::vector<std::string> to_remove = {obj_id};
          planning_scene_interface->removeCollisionObjects(to_remove);
          ros::Duration(0.1).sleep();
        }

        addCollisionObject(obj_id, primitive, obj_pose, BASE_FRAME);
        detected_tags[tag_id] = entry.pose;
        last_object_update[tag_id] = now;
      }
    }
  }

  // Clean up objects that are no longer detected
  std::vector<std::string> to_remove;
  for (auto it = detected_tags.begin(); it != detected_tags.end();)
  {
    int id = it->first;

    if (!seen_ids.count(id))
    {
      // Object not detected anymore

      // Don't remove if it's currently being manipulated
      if (currently_manipulated_id == id)
      {
        ROS_DEBUG("Keeping manipulated object %d in scene despite not being detected", id);
        ++it;
        continue;
      }

      // Remove from scene and cleanup timestamp
      std::string obj_id = getCollisionObjectId(id);
      to_remove.push_back(obj_id);
      last_object_update.erase(id); // Clean up timestamp tracking
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
    ROS_DEBUG("Removed %zu collision objects from scene", to_remove.size());
  }
}

bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  ROS_INFO("========== PICK OBJECT SERVICE CALLED ==========");
  ROS_INFO("Target ID: %d", req.target_id);

  int id = req.target_id;

  // STEP 0: Freeze object updates for this object
  currently_manipulated_id = id;
  ROS_INFO("Freezing updates for object ID %d", id);

  try
  {
    // Initialize gripper controller
    gripper_control::GripperController gripper("gripper");
    gripper.registerNamedTarget("open", 0.044);
    gripper.registerNamedTarget("closed", 0.0);

    // Open gripper before approach
    ROS_INFO("Opening gripper before approach");
    gripper.open();

    // Set end effector link
    arm_group->setEndEffectorLink("gripper_grasping_frame");

    // Get target pose
    geometry_msgs::Pose target_pose = req.target.pose;

    // Set the collision object ID for later attachment
    attached_object_id = getCollisionObjectId(id);

    // Calculate approach pose based on object type
    geometry_msgs::Pose above_pose;
    tf2::Quaternion final_orientation_q;
    double object_height;

    if (id >= 1 && id <= 3)
    {
      // Cylinders
      ROS_INFO("Handling cylinder (ID %d)", id);
      object_height = 0.1;

      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, 0); // Point down
      final_orientation_q = downward_q;

      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 4 && id <= 6)
    {
      // Cubes
      ROS_INFO("Handling cube (ID %d)", id);
      object_height = 0.0495;

      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(target_q).getRPY(roll, pitch, yaw);

      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, yaw); // Preserve yaw, point down
      final_orientation_q = downward_q;

      above_pose.position = target_pose.position;
      above_pose.position.z += 0.35;
    }
    else if (id >= 7 && id <= 9)
    {
      // Prisms
      ROS_INFO("Handling prism (ID %d)", id);
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
      currently_manipulated_id = -1; // Reset freeze
      res.success = false;
      return true;
    }

    // Apply gripper offset and set orientation
    above_pose.orientation = tf2::toMsg(final_orientation_q);
    tf2::Vector3 gripper_offset(-0.05, 0.0, 0.0);
    tf2::Vector3 offset_world = tf2::quatRotate(final_orientation_q, gripper_offset);
    above_pose.position.x += offset_world.x();
    above_pose.position.y += offset_world.y();
    above_pose.position.z += offset_world.z();

    // STEP 1: Move above object with improved planning (object exists for collision checking)
    ROS_INFO("STEP 1: Moving above object (collision object exists for planning)");
    ROS_INFO("Target position: x=%.3f, y=%.3f, z=%.3f",
             above_pose.position.x, above_pose.position.y, above_pose.position.z);

    // Set planning parameters for robust operation
    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setGoalPositionTolerance(0.010);   // 10 mm
    arm_group->setGoalOrientationTolerance(0.03); // ~1.7°
    arm_group->setPlanningTime(10.0);
    arm_group->setNumPlanningAttempts(10);
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    arm_group->setPoseTarget(above_pose);
    arm_group->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    if (arm_group->plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Initial planning attempt failed, trying with relaxed tolerances...");

      // Try with more relaxed tolerances
      arm_group->setGoalPositionTolerance(0.015);
      arm_group->setGoalOrientationTolerance(0.015);
      arm_group->setPlanningTime(15.0);
      arm_group->setNumPlanningAttempts(20);

      if (arm_group->plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR("Failed to plan movement above object!");
        currently_manipulated_id = -1;
        res.success = false;
        return true;
      }
    }

    if (arm_group->execute(plan1) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to move above object!");
      currently_manipulated_id = -1;
      res.success = false;
      return true;
    }

    arm_group->clearPoseTargets();
    ROS_INFO("Successfully moved above target!");
    ros::Duration(0.5).sleep();
    // STEP 2: Cartesian move down to grasp (object still exists)
    ROS_INFO("STEP 2: Moving down to grasp position");
    geometry_msgs::Pose grasp_pose = above_pose;
    grasp_pose.position.z = target_pose.position.z - (object_height / 2.0) + 0.025;

    ROS_INFO("Moving to grasp position at z=%.3f (target_z=%.3f, object_height=%.3f)",
             grasp_pose.position.z, target_pose.position.z, object_height);

    bool grasp_motion_success = false;

    // ATTEMPT 1: Try Cartesian path first (preferred for smooth motion)
    ROS_INFO("STEP 2a: Attempting Cartesian path down to grasp");

    // Create waypoints for Cartesian path
    std::vector<geometry_msgs::Pose> waypoints = {above_pose, grasp_pose};
    moveit_msgs::RobotTrajectory trajectory;

    // Plan Cartesian path with explicit parameters
    const double eef_step = 0.015; // Fine interpolation helps IK continuity
    const bool avoid_collisions = true;
    arm_group->setStartStateToCurrentState();

    moveit_msgs::MoveItErrorCodes cart_err;
    double fraction = arm_group->computeCartesianPath(
        waypoints, eef_step, trajectory, avoid_collisions, &cart_err);

    if (cart_err.val != moveit_msgs::MoveItErrorCodes::SUCCESS && fraction >= 0.65)
    {
      ROS_WARN("computeCartesianPath returned non-success code %d but acceptable fraction=%.2f",
               cart_err.val, fraction);
    }

    if (fraction >= 0.85)
    {
      ROS_INFO("Cartesian path planning successful (%.2f%% coverage)", fraction * 100);

      // Time-parameterize before execution for smoother motion
      robot_trajectory::RobotTrajectory rt(
          arm_group->getCurrentState()->getRobotModel(), arm_group->getName());
      rt.setRobotTrajectoryMsg(*arm_group->getCurrentState(), trajectory);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      bool timing_ok = iptp.computeTimeStamps(rt, 0.3 /*vel*/, 0.3 /*acc*/);
      if (!timing_ok)
      {
        ROS_WARN("Time-parameterization failed; proceeding with un-timed trajectory may be unstable");
      }
      rt.getRobotTrajectoryMsg(trajectory);

      // Execute the Cartesian path
      moveit::planning_interface::MoveGroupInterface::Plan plan2;
      plan2.trajectory_ = trajectory;

      if (arm_group->execute(plan2) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Cartesian path execution successful!");
        grasp_motion_success = true;
      }
      else
      {
        ROS_WARN("Cartesian path execution failed, will try regular planning fallback");
        grasp_motion_success = false;
      }
    }
    else
    {
      ROS_WARN("Cartesian path planning failed (only %.2f%% achieved), will try regular planning fallback",
               fraction * 100);
      grasp_motion_success = false;
    }

    // ATTEMPT 2: Fallback to regular motion planning if Cartesian failed
    if (!grasp_motion_success)
    {
      ROS_INFO("STEP 2b: Attempting regular motion planning fallback to grasp");

      // Set planning parameters for robust operation
      arm_group->setPlannerId("RRTConnectkConfigDefault");
      arm_group->setGoalPositionTolerance(0.010);   // 10 mm
      arm_group->setGoalOrientationTolerance(0.03); // ~1.7°
      arm_group->setPlanningTime(10.0);
      arm_group->setNumPlanningAttempts(10);
      arm_group->setMaxVelocityScalingFactor(0.3); // Slower for precision
      arm_group->setMaxAccelerationScalingFactor(0.3);

      arm_group->setPoseTarget(grasp_pose);
      arm_group->setStartStateToCurrentState();

      moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;

      if (arm_group->plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Regular planning successful, executing...");

        if (arm_group->execute(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Regular planning execution successful!");
          grasp_motion_success = true;
        }
        else
        {
          ROS_WARN("Regular planning execution failed, trying with relaxed tolerances...");

          // Try with more relaxed tolerances
          arm_group->setGoalPositionTolerance(0.015);
          arm_group->setGoalOrientationTolerance(0.05);
          arm_group->setPlanningTime(15.0);
          arm_group->setNumPlanningAttempts(20);

          if (arm_group->plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
          {
            if (arm_group->execute(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
              ROS_INFO("Relaxed tolerance regular planning successful!");
              grasp_motion_success = true;
            }
            else
            {
              ROS_ERROR("All regular planning execution attempts failed!");
            }
          }
          else
          {
            ROS_ERROR("All regular planning attempts failed!");
          }
        }
      }
      else
      {
        ROS_ERROR("Regular planning failed completely!");
      }
    }

    // Check if any method succeeded
    if (!grasp_motion_success)
    {
      ROS_ERROR("Both Cartesian and regular planning failed to reach grasp position!");
      currently_manipulated_id = -1;
      res.success = false;
      return true;
    }

    arm_group->clearPoseTargets();
    ROS_INFO("Successfully reached grasp position!");

    // Wait for robot to settle
    ros::Duration(1.0).sleep();
    // STEP 3: Attach object, THEN remove from scene
    ROS_INFO("STEP 3a: Attaching object to gripper in Gazebo");

    std::string gazebo_attach_service = "/link_attacher_node/attach";
    if (ros::service::waitForService(gazebo_attach_service, ros::Duration(2.0)))
    {
      ros::ServiceClient attach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(gazebo_attach_service);
      gazebo_ros_link_attacher::Attach attach_srv;
      attach_srv.request.model_name_1 = "tiago";
      attach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      // Set model names based on ID
      std::string object_model_name, object_link_name;
      if (id == 1)
      {
        object_model_name = "Hexagon";
        object_link_name = "Hexagon_link";
      }
      else if (id == 2)
      {
        object_model_name = "Hexagon_2";
        object_link_name = "Hexagon_2_link";
      }
      else if (id == 3)
      {
        object_model_name = "Hexagon_3";
        object_link_name = "Hexagon_3_link";
      }
      else if (id == 4)
      {
        object_model_name = "cube";
        object_link_name = "cube_link";
      }
      else if (id == 5)
      {
        object_model_name = "cube_5";
        object_link_name = "cube_5_link";
      }
      else if (id == 6)
      {
        object_model_name = "cube_6";
        object_link_name = "cube_6_link";
      }
      else if (id == 7)
      {
        object_model_name = "Triangle";
        object_link_name = "Triangle_link";
      }
      else if (id == 8)
      {
        object_model_name = "Triangle_8";
        object_link_name = "Triangle_8_link";
      }
      else if (id == 9)
      {
        object_model_name = "Triangle_9";
        object_link_name = "Triangle_9_link";
      }

      attach_srv.request.model_name_2 = object_model_name;
      attach_srv.request.link_name_2 = object_link_name;

      if (attach_client.call(attach_srv) && attach_srv.response.ok)
      {
        ROS_INFO("Successfully attached object in Gazebo physics");
      }
      else
      {
        ROS_WARN("Failed to attach in Gazebo - continuing anyway");
      }
    }
    else
    {
      ROS_WARN("Gazebo attach service not available");
    }

    // STEP 3b: Attach in MoveIt planning scene
    ROS_INFO("STEP 3b: Attaching object in MoveIt planning scene");

    moveit_msgs::AttachedCollisionObject attached_co;
    attached_co.link_name = MOVEIT_ATTACH_LINK;
    attached_co.object.header.frame_id = MOVEIT_ATTACH_LINK;
    attached_co.object.id = attached_object_id;

    // Set the pose relative to the gripper frame
    geometry_msgs::Pose attach_pose;
    attach_pose.orientation.w = 1.0;
    attach_pose.position.x = -0.05; // Gripper offset
    attach_pose.position.y = 0.0;
    attach_pose.position.z = 0.0;

    // Add both pose arrays for compatibility
    attached_co.object.primitive_poses.push_back(attach_pose);
    attached_co.object.mesh_poses.push_back(attach_pose);

    // Touch links to avoid self-collision
    attached_co.touch_links = {
        "gripper_left_finger_link",
        "gripper_right_finger_link",
        "gripper_grasping_frame",
        "gripper_grasping_frame_Y",
        "gripper_grasping_frame_Z",
        "arm_7_link", "arm_6_link", "arm_5_link", "arm_4_link", "arm_3_link"};

    attached_co.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface->applyAttachedCollisionObject(attached_co);

    ROS_INFO("STEP 3c: NOW removing collision object from scene");
    // NOW remove the standalone collision object since it's attached
    std::vector<std::string> to_remove_after_attach = {attached_object_id};
    planning_scene_interface->removeCollisionObjects(to_remove_after_attach);
    ros::Duration(0.5).sleep();

    ROS_INFO("Object attached and removed from scene");

    // STEP 4: Close gripper (object should now be attached in Gazebo)
    ROS_INFO("STEP 4: Closing gripper");
    gripper.close(id);
    ros::Duration(0.5).sleep();

    // STEP 5: Lift object with standard motion planning
    ROS_INFO("STEP 5: Lifting object");
    geometry_msgs::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.3;
    ROS_INFO("Lifting to position at z=%.3f", lift_pose.position.z);

    // Set planning parameters for robust operation
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
      if (arm_group->execute(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Standard lift planning successful!");
      }
      else
      {
        ROS_ERROR("Standard lift execution failed!");
        res.success = false;
        return true;
      }
    }
    else
    {
      ROS_WARN("Standard lift planning failed, trying with relaxed tolerances...");

      // Try with more relaxed tolerances
      arm_group->setGoalPositionTolerance(0.02);
      arm_group->setGoalOrientationTolerance(0.02);
      arm_group->setPlanningTime(20.0);

      if (arm_group->plan(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        if (arm_group->execute(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Relaxed tolerance lift successful!");
        }
        else
        {
          ROS_ERROR("Relaxed tolerance lift execution failed!");
          res.success = false;
          return true;
        }
      }
      else
      {
        ROS_ERROR("All lift planning attempts failed!");
        res.success = false;
        return true;
      }
    }
    ROS_INFO("Successfully lifted object!");
    ros::Duration(1.0).sleep(); // Let movement settle

    // STEP 6: Move arm to safe position for travel  ← WRONG LOCATION
    ROS_INFO("STEP 6: Moving arm to safe position");
    moveArmToSafePosition();

    ROS_INFO("Pick sequence completed successfully!");
    res.success = true;
    // NOTE: Keep currently_manipulated_id set until after placement!
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in pickObjectCallback: %s", e.what());
    currently_manipulated_id = -1; // Reset freeze on error
    res.success = false;
  }

  return true;
}

// ==================== COMPLETE PLACE OBJECT CALLBACK WITH ARM MOVEMENT ====================
bool placeObjectCallback(assignment2_package::PlaceObject::Request &req,
                         assignment2_package::PlaceObject::Response &res)
{
  ROS_INFO("========== PLACE OBJECT SERVICE CALLED ==========");
  ROS_INFO("Target ID: %d", req.target_id);

  try
  {
    int id = req.target_id;

    // STEP 1: Get the placement pose from Node A and add object-specific Z offset
    ROS_INFO("STEP 1: Adding object-specific Z offset to placement pose from Node A");

    // Determine Z offset based on object ID
    double z_offset;
    if (id >= 1 && id <= 3)
    {
      z_offset = 0.15; // Cylinders get 0.13m offset
      ROS_INFO("Using 0.13 Z offset for cylinder (ID %d)", id);
    }
    else if (id >= 4 && id <= 9)
    {
      z_offset = 0.06; // Cubes and prisms get 0.06m offset
      ROS_INFO("Using 0.06m Z offset for cube/prism (ID %d)", id);
    }
    else
    {
      ROS_ERROR("Invalid object ID: %d", id);
      res.success = false;
      return true;
    }

    // Get the base placement pose from Node A (already in base_footprint frame)
    geometry_msgs::PoseStamped placement_pose = req.target_pose;
    double table_surface_z = placement_pose.pose.position.z;

    // Add the object-specific Z offset
    placement_pose.pose.position.z = table_surface_z + z_offset;
    broadcastPlacementPose(placement_pose, "placement_pose_frame");
    ROS_INFO("Broadcasting placement pose frame for RViz visualization");

    ROS_INFO("Received base placement pose: base(%.3f, %.3f, %.3f)",
             req.target_pose.pose.position.x,
             req.target_pose.pose.position.y,
             req.target_pose.pose.position.z);
    ROS_INFO("Final placement pose with offset: base(%.3f, %.3f, %.3f) [table=%.3f + offset=%.2f]",
             placement_pose.pose.position.x,
             placement_pose.pose.position.y,
             placement_pose.pose.position.z,
             table_surface_z, z_offset);

    // STEP 2: Move arm to placement position with offset
    ROS_INFO("STEP 2: Moving arm to placement position with object-specific offset");

    // Configure arm_torso for placement movement
    moveit::planning_interface::MoveGroupInterface arm_torso_group("arm_torso");
    arm_torso_group.setEndEffectorLink("gripper_grasping_frame");
    arm_torso_group.setPoseTarget(placement_pose.pose, "gripper_grasping_frame");

    // Set planning parameters
    arm_torso_group.setPlanningTime(30.0);
    arm_torso_group.setNumPlanningAttempts(20);
    arm_torso_group.setGoalPositionTolerance(0.05);
    arm_torso_group.setGoalOrientationTolerance(0.02);
    arm_torso_group.setMaxVelocityScalingFactor(0.5);
    arm_torso_group.setMaxAccelerationScalingFactor(0.5);

    moveit::planning_interface::MoveGroupInterface::Plan placement_plan;
    bool success = (arm_torso_group.plan(placement_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      ROS_ERROR("Failed to plan movement to placement position. Trying alternative approach...");

      // Try with arm_tool_link as end effector
      arm_torso_group.setEndEffectorLink("arm_tool_link");
      arm_torso_group.setPoseTarget(placement_pose.pose, "arm_tool_link");
      success = (arm_torso_group.plan(placement_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!success)
      {
        ROS_ERROR("Failed to plan placement movement with both end effectors!");
        res.success = false;
        return true;
      }
    }

    ROS_INFO("Executing movement to placement position...");
    moveit::core::MoveItErrorCode exec_result = arm_torso_group.execute(placement_plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Movement execution returned error %d, continuing with placement...", exec_result.val);
    }

    ros::Duration(2.0).sleep(); // Allow settling

    // STEP 3: Detach from MoveIt planning scene
    ROS_INFO("STEP 3: Detaching from MoveIt planning scene");
    moveit_msgs::AttachedCollisionObject detach_from_moveit;
    detach_from_moveit.link_name = MOVEIT_ATTACH_LINK;
    detach_from_moveit.object.id = attached_object_id;
    detach_from_moveit.object.operation = moveit_msgs::CollisionObject::REMOVE;
    detach_from_moveit.object.header.frame_id = MOVEIT_ATTACH_LINK;
    planning_scene_interface->applyAttachedCollisionObject(detach_from_moveit);
    ros::Duration(0.5).sleep();

    // Remove object to avoid issues with gripper
    if (!attached_object_id.empty())
    {
      std::vector<std::string> rm_ids = {attached_object_id};
      planning_scene_interface->removeCollisionObjects(rm_ids);
      ROS_INFO("Removed world collision object '%s' (temporarily) to avoid gripper start-in-collision.",
               attached_object_id.c_str());
      ros::Duration(0.2).sleep();
    }

    // STEP 4: Detach from Gazebo physics BEFORE opening gripper
    ROS_INFO("STEP 4: Detaching from Gazebo physics");
    std::string gazebo_detach_service = "/link_attacher_node/detach";
    if (ros::service::waitForService(gazebo_detach_service, ros::Duration(2.0)))
    {
      ros::ServiceClient detach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(gazebo_detach_service);
      gazebo_ros_link_attacher::Attach detach_srv;
      detach_srv.request.model_name_1 = "tiago";
      detach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      // Set model names based on ID
      std::string object_model_name, object_link_name;
      if (id == 1)
      {
        object_model_name = "Hexagon";
        object_link_name = "Hexagon_link";
      }
      else if (id == 2)
      {
        object_model_name = "Hexagon_2";
        object_link_name = "Hexagon_2_link";
      }
      else if (id == 3)
      {
        object_model_name = "Hexagon_3";
        object_link_name = "Hexagon_3_link";
      }
      else if (id == 4)
      {
        object_model_name = "cube";
        object_link_name = "cube_link";
      }
      else if (id == 5)
      {
        object_model_name = "cube_5";
        object_link_name = "cube_5_link";
      }
      else if (id == 6)
      {
        object_model_name = "cube_6";
        object_link_name = "cube_6_link";
      }
      else if (id == 7)
      {
        object_model_name = "Triangle";
        object_link_name = "Triangle_link";
      }
      else if (id == 8)
      {
        object_model_name = "Triangle_8";
        object_link_name = "Triangle_8_link";
      }
      else if (id == 9)
      {
        object_model_name = "Triangle_9";
        object_link_name = "Triangle_9_link";
      }

      detach_srv.request.model_name_2 = object_model_name;
      detach_srv.request.link_name_2 = object_link_name;

      if (detach_client.call(detach_srv) && detach_srv.response.ok)
      {
        ROS_INFO("Detached object from Gazebo");
      }
      else
      {
        ROS_WARN("Failed to detach from Gazebo");
      }
    }

    // Give some time for the physics to settle after detachment
    ros::Duration(1.0).sleep();

    // STEP 5: Open gripper (object is detached from both MoveIt and Gazebo)
    ROS_INFO("STEP 5: Opening gripper (object now detached from Gazebo)");
    gripper_control::GripperController gripper("gripper");
    gripper.registerNamedTarget("open", 0.044);
    gripper.open();
    ros::Duration(0.5).sleep();

    // STEP 6: Small retreat from placement
    ROS_INFO("STEP 6: Small retreat from placement");
    geometry_msgs::PoseStamped current_pose_stamped = arm_group->getCurrentPose();
    geometry_msgs::Pose retreat_pose = current_pose_stamped.pose;
    retreat_pose.position.z += 0.1;

    arm_group->setPoseTarget(retreat_pose);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.3);

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    if (arm_group->plan(retreat_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      arm_group->execute(retreat_plan);
      ROS_INFO("Successfully retreated slightly");
    }
    else
    {
      ROS_WARN("Could not retreat, but object was placed");
    }

    arm_group->setMaxVelocityScalingFactor(0.9);
    arm_group->setMaxAccelerationScalingFactor(0.9);

    // STEP 6.5 MOVE ARM TO SAFE POSITION
    ROS_INFO("STEP 6.5: Moving arm to safe position before unfreezing object");
    moveArmToSafePosition(); // Don't check return value - continue regardless

    // STEP 7: CRITICAL - Unfreeze the object for future collision avoidance
    ROS_INFO("STEP 7: Unfreezing object %d for collision detection", id);
    currently_manipulated_id = -1; // Reset to allow updates
    frozen_objects.erase(id);      // Remove from frozen list
    attached_object_id = "";       // Clear attached ID

    // The next objectPosesCallback will re-add this object to the scene
    ROS_INFO("Object %d will be re-added to collision scene on next update", id);

    ROS_INFO("Simplified place sequence with offset completed successfully!");
    res.success = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in placeObjectCallback: %s", e.what());

    // Reset state on error
    currently_manipulated_id = -1;
    attached_object_id = "";

    res.success = false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_C");
  ros::NodeHandle nh;
  nh_ptr = &nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize TF
  tf2_ros::Buffer tfBuffer_obj;
  tf2_ros::TransformListener tfListener_obj(tfBuffer_obj);
  tfBuffer = &tfBuffer_obj;
  tfListener = &tfListener_obj;

  // Initialize MoveIt
  try
  {
    ROS_INFO("Initializing MoveIt interfaces...");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    const std::vector<std::string> &group_names = kinematic_model->getJointModelGroupNames();

    ROS_INFO("Available planning groups:");
    for (const std::string &group_name : group_names)
    {
      ROS_INFO("  - %s", group_name.c_str());
    }

    static moveit::planning_interface::MoveGroupInterface arm_mg(PLANNING_GROUP_ARM);
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
    ROS_ERROR("Failed to initialize MoveIt: %s", e.what());
    return -1;
  }

  // Setup subscribers and services
  ros::Subscriber object_sub = nh.subscribe("object_poses", 1, objectPosesCallback);
  get_obj_pose_client = nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
  ros::ServiceServer pick_service = nh.advertiseService("pick_object", pickObjectCallback);
  ros::ServiceServer place_service = nh.advertiseService("place_object", placeObjectCallback);

  ROS_INFO("Node C ready - Object freezing logic enabled");
  ros::waitForShutdown();

  return 0;
}