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
#include "gripper_control.h"
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
#include <memory>

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

// Keep track of which collision objects are currently in the scene
std::map<int, geometry_msgs::PoseStamped> detected_tags;
std::map<int, bool> persistent_tables; // Track which tables have been created

// New global flag to manage object attachment state
bool is_object_attached = false;
std::string attached_object_id;
double saved_pickup_z = 0.0;

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
  ros::Duration(0.1).sleep();
  ros::spinOnce();   
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

  // Load & scale the mesh with RAII
  std::unique_ptr<shapes::Mesh> m(
      static_cast<shapes::Mesh *>(shapes::createMeshFromResource(mesh_resource, scale)));

  // Convert to the ShapeMsg variant and extract the mesh
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m.get(), shape_msg);
  const shape_msgs::Mesh &mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Populate and apply
  co.meshes.push_back(mesh_msg);
  co.mesh_poses.push_back(pose);
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface->applyCollisionObjects({co});
  ros::Duration(0.1).sleep();
  ros::spinOnce();   
}

// Subscriber callback: update scene from ObjectPoseArray
void objectPosesCallback(const assignment2_package::ObjectPoseArray::ConstPtr &msg)
{
  std::set<int> seen_ids;

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

    // Skip tag 10 - it's the reference tag on the placing table
    if (tag_id == 10)
    {
      ROS_DEBUG("Skipping tag 10 (reference tag) - no collision object needed");
      continue;
    }
    // CRITICAL FIX: Skip creating collision objects for attached objects
    std::string current_obj_id = (tag_id >= 7 && tag_id <= 9)
                                     ? "tag_mesh_" + std::to_string(tag_id)
                                     : "tag_object_" + std::to_string(tag_id);

    if (is_object_attached && current_obj_id == attached_object_id)
    {
      ROS_DEBUG("Skipping collision object creation for attached object: %s", current_obj_id.c_str());
      seen_ids.insert(tag_id); // Mark as seen so it doesn't get removed
      continue;
    }
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

      std::string obj_id = "tag_mesh_" + std::to_string(tag_id);


      // Now add with new pose
      addMeshCollisionObject(obj_id, mesh_uri, obj_pose, BASE_FRAME, prism_scale);
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
    else if (tag_id >= 4 && tag_id <= 6)
    {
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.05, 0.05, 0.05};     // x, y, z
      height_offset = primitive.dimensions[2] / 2.0; // half of box height
    }
    else
    {
      // Any other tag ID that's not 10 and not in the defined ranges
      ROS_WARN("Unknown tag ID %d detected - skipping collision object creation", tag_id);
      continue;
    }

    // Adjust Z position: move down by half the object height so bottom sits on surface
    obj_pose.position.z -= height_offset + 0.005;

    std::string obj_id = "tag_object_" + std::to_string(tag_id);
    addCollisionObject(obj_id, primitive, obj_pose, BASE_FRAME);
    detected_tags[tag_id] = entry.pose;
  }

  // Removal logic: ONLY remove objects if the robot is not holding one
  std::vector<std::string> to_remove;
  for (auto it = detected_tags.begin(); it != detected_tags.end();)
  {
    int id = it->first;
    std::string obj_id = (id >= 7 && id <= 9)
                             ? "tag_mesh_" + std::to_string(id)
                             : "tag_object_" + std::to_string(id);

    if (!seen_ids.count(id))
    {
      // Object not detected by vision anymore

      // If we're holding an object, only remove if it's NOT the attached object
      if (is_object_attached)
      {
        if (obj_id != attached_object_id)
        {
          to_remove.push_back(obj_id);
          it = detected_tags.erase(it);
        }
        else
        {
          // Keep the attached object in detected_tags but don't remove from scene
          ++it;
        }
      }
      else
      {
        // Not holding anything, remove normally
        to_remove.push_back(obj_id);
        it = detected_tags.erase(it);
      }
    }
    else
    {
      ++it;
    }
  }

  if (!to_remove.empty())
  {
    planning_scene_interface->removeCollisionObjects(to_remove);
    ros::Duration(0.1).sleep();
    ros::spinOnce();   
  }
}

bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  // Set flag to true to prevent object removal from planning scene
  is_object_attached = true;
  auto reset_flag = [&]()
  {
    is_object_attached = false;
    attached_object_id.clear();
  };
  ROS_INFO("PickObject service called - executing pick sequence");
  try
  {
    // Initialize gripper controller (assuming "gripper" is the planning group name)
    gripper_control::GripperController gripper("gripper");

    // Register named targets for gripper
    gripper.registerNamedTarget("open", 0.044); // Max opening
    gripper.registerNamedTarget("closed", 0.0); // Fully closed

    // Open gripper before approaching
    ROS_INFO("Opening gripper before approach");
    gripper.open();

    // Set the end effector link
    arm_group->setEndEffectorLink("gripper_grasping_frame");
    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setGoalPositionTolerance(0.010);   // 10 mm
    arm_group->setGoalOrientationTolerance(0.03); // ~1.7°
    arm_group->setPlanningTime(2.0);
    arm_group->setNumPlanningAttempts(10);

    // Get the target pose from the request
    geometry_msgs::Pose target_pose = req.target.pose;

    // STEP 1: Calculate the above_pose and object_height based on the ID
    geometry_msgs::Pose above_pose;
    tf2::Quaternion final_orientation_q;
    double object_height;
    int id = req.target_id;

    // Set the collision object ID to use for attachment
    if (id >= 7 && id <= 9)
    {
      attached_object_id = "tag_mesh_" + std::to_string(id);
    }
    else
    {
      attached_object_id = "tag_object_" + std::to_string(id);
    }

    if (id >= 1 && id <= 3)
    {
      // Case 1: For cylinders - USE FLIPPED ORIENTATION
      ROS_INFO("Using flipped gripper orientation for cylinder (ID %d)", id);
      object_height = 0.1;

      // This creates a quaternion where Z points straight down
      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, 0); // 180° rotation around X makes Z point down
      final_orientation_q = downward_q;

      // Start from target position and move up 0.2m in world Z
      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 4 && id <= 6)
    {
      // Case 2: For cubes - USE FLIPPED ORIENTATION
      ROS_INFO("Using flipped gripper orientation for cube (ID %d)", id);
      object_height = 0.0495;

      // Get target orientation
      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);

      // Since cubes might be rotated, preserve their yaw but flip to point down
      double roll, pitch, yaw;
      tf2::Matrix3x3(target_q).getRPY(roll, pitch, yaw);

      // Create downward-pointing orientation with object's yaw
      tf2::Quaternion downward_q;
      downward_q.setRPY(M_PI, 0, yaw);
      final_orientation_q = downward_q;

      // Start from target position and move up 0.2m in world Z
      above_pose.position = target_pose.position;
      above_pose.position.z += 0.2;
    }
    else if (id >= 7 && id <= 9)
    {
      // Case 3: For prism on slope — requested orientation sequence:
      // final = target_q * Rx(+45°) * Rx(180°) * Rz(+90°)
      ROS_INFO("Handling prism on slope (IDs 7-9) with specific above-pose orientation");
      object_height = 0.032;

      // Start from the measured tag pose
      tf2::Quaternion target_q;
      tf2::fromMsg(target_pose.orientation, target_q);

      // Build the incremental rotations
      tf2::Quaternion q_rx45;
      q_rx45.setRPY(M_PI / 4, 0, 0); // +45° about X
      tf2::Quaternion q_rx180;
      q_rx180.setRPY(M_PI, 0, 0); // +180° about X
      tf2::Quaternion q_rz90;
      q_rz90.setRPY(0, 0, M_PI / 2); // +90° about Z

      // Compose in LOCAL frame (right-multiply)
      final_orientation_q = target_q * q_rx45 * q_rx180 * q_rz90;
      final_orientation_q.normalize();

      // "Above" point: back off 0.20 m along the tool -Z of the FINAL orientation
      tf2::Vector3 approach_offset(0, 0, -0.2);
      tf2::Vector3 offset_world = tf2::quatRotate(final_orientation_q, approach_offset);

      above_pose.position = target_pose.position;
      above_pose.position.x += offset_world.x();
      above_pose.position.y += offset_world.y();
      above_pose.position.z += offset_world.z();

      // (Orientation is assigned after the if/else via final_orientation_q)
      ROS_INFO("Prism above-pose set with Rx(+45)->Rx(180)->Rz(+90) from tag frame");
    }
    else
    {
      ROS_ERROR("Invalid object ID: %d", id);
      res.success = false;
      reset_flag();
      return true;
    }

    // Set the final orientation
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
    arm_group->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm_group->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success1)
    {
      ROS_ERROR("Failed to plan movement above object with adjusted orientation!");
      res.success = false;
      reset_flag();
      return true;
    }

    moveit::core::MoveItErrorCode execute_result1 = arm_group->execute(plan1);
    if (execute_result1 != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to move above object!");
      res.success = false;
      reset_flag();
      return true;
    }

    arm_group->clearPoseTargets();

    ROS_INFO("Successfully moved above target!");

    // STEP 2: Cartesian move down
    ROS_INFO("STEP 2: Moving down to grasp position with Cartesian path");

    // Calculate the grasp position
    geometry_msgs::Pose grasp_pose = above_pose; // Start with above pose (keeps orientation)

    // Move down to target Z position minus half object height, plus 0.05 for finger clearance
    grasp_pose.position.z = target_pose.position.z - (object_height / 2.0) + 0.05;

    ROS_INFO("Moving to grasp position at z=%.3f (target_z=%.3f, object_height=%.3f)",
             grasp_pose.position.z, target_pose.position.z, object_height);

    // Create waypoints for Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(above_pose); // Start position
    waypoints.push_back(grasp_pose); // End position

    // Plan Cartesian path (explicit jump threshold + collision avoidance)
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.005;      // finer interpolation helps IK continuity
    const bool avoid_collisions = true; // keep true while object is (or will be) attached
    
    arm_group->stop();  
    arm_group->setStartStateToCurrentState();
    ros::Duration(0.05).sleep(); 
  

    moveit_msgs::MoveItErrorCodes cart_err;
    double fraction = arm_group->computeCartesianPath(
        waypoints, eef_step, trajectory, avoid_collisions, &cart_err);
    if (cart_err.val != moveit_msgs::MoveItErrorCodes::SUCCESS && fraction >= 0.65)
    {
      ROS_WARN("computeCartesianPath returned non-success code %d but acceptable fraction=%.2f",
               cart_err.val, fraction);
    }
    if (fraction < 0.65) // If less than 65% of path achieved
    {
      ROS_ERROR("Could not plan full Cartesian path down to object!");
      res.success = false;
      reset_flag();
      return true;
    }

    // Time-parameterize before execution (adds timestamps/vel/acc)
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

    moveit::core::MoveItErrorCode execute_result2 = arm_group->execute(plan2);

    if (execute_result2 != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to execute Cartesian move down!");
      res.success = false;
      reset_flag();
      return true;
    }

    arm_group->clearPoseTargets();

    ROS_INFO("Successfully moved to grasp position!");

    // *** SAVE THE PICKUP Z COORDINATE FOR PLACEMENT ***
    saved_pickup_z = grasp_pose.position.z + 0.03;
    ROS_INFO("Saved pickup Z coordinate: %.3f for future placement", saved_pickup_z);

    // Wait for robot to settle and physics to stabilize
    ros::Duration(1.0).sleep();

    // STEP 3: Attach object to gripper (GAZEBO & MOVEIT!)

    // First, do the Gazebo link attachment
    ROS_INFO("STEP 3a: Attaching object to gripper in Gazebo");
    std::string gazebo_attach_service = "/link_attacher_node/attach";
    bool gazebo_service_exists = ros::service::waitForService(gazebo_attach_service, ros::Duration(2.0));

    if (gazebo_service_exists)
    {
      ros::ServiceClient attach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(gazebo_attach_service);
      gazebo_ros_link_attacher::Attach attach_srv;
      attach_srv.request.model_name_1 = "tiago";
      attach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      std::string object_model_name;
      std::string object_link_name;

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

      if (attach_client.call(attach_srv))
      {
        if (attach_srv.response.ok)
        {
          ROS_INFO("Successfully attached object %s to gripper in Gazebo", object_model_name.c_str());
        }
        else
        {
          ROS_WARN("Failed to attach object in Gazebo");
        }
      }
      else
      {
        ROS_WARN("Failed to call Gazebo attach service - continuing without attachment");
      }
    }
    else
    {
      ROS_WARN("Gazebo link attacher service not available - continuing without attachment");
    }

    // Now, attach the object in MoveIt!'s planning scene
    ROS_INFO("STEP 3b: Attaching object to gripper in MoveIt!");
    moveit_msgs::AttachedCollisionObject attached_co;
    attached_co.link_name = MOVEIT_ATTACH_LINK;

    // Just reference the SAME id; MoveIt will move it from world -> attached.
    attached_co.object.id = attached_object_id;
    attached_co.object.header.frame_id = MOVEIT_ATTACH_LINK;

    planning_scene_interface->removeCollisionObjects({attached_object_id}); 
    ros::Duration(0.1).sleep();
    ros::spinOnce();   
      
    // Optional: keep the small offset so the collision geometry sits inside the fingers
    geometry_msgs::Pose attach_pose;
    attach_pose.orientation.w = 1.0; // aligned with link frame
    attach_pose.position.x = -0.05;  // into the fingers

    // You can provide both pose arrays; harmless if one isn't used
    attached_co.object.primitive_poses.push_back(attach_pose);
    attached_co.object.mesh_poses.push_back(attach_pose);

    // Touch links so the attached object can contact the gripper without collisions
    attached_co.touch_links = {
        "gripper_left_finger_link",
        "gripper_right_finger_link",
        "gripper_grasping_frame",
        "gripper_grasping_frame_Y",
        "gripper_grasping_frame_Z"};

    // This is the crucial bit: ADD with an existing id attaches it (and removes from world)
    attached_co.object.operation = moveit_msgs::CollisionObject::ADD;

    planning_scene_interface->applyAttachedCollisionObject(attached_co);
    ros::Duration(0.1).sleep();
    ros::spinOnce();   
    ROS_INFO("Successfully attached object '%s' to gripper link '%s' in MoveIt!",
             attached_object_id.c_str(), attached_co.link_name.c_str());

    // Wait for attachment to reflect in the planning scene
    ros::Duration(1.0).sleep();

    // STEP 4: Close the gripper
    ROS_INFO("STEP 4: Closing gripper for object ID %d", id);

    try
    {
      // Close gripper with appropriate width for the object
      gripper.close(id);

      // Small delay to ensure gripper has closed
      ros::Duration(0.5).sleep();

      ROS_INFO("Gripper closed successfully!");
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Failed to close gripper: %s", e.what());
      reset_flag();
      res.success = false;
      return true;
    }

    // STEP 5: Lift object straight up
    ROS_INFO("STEP 5: Lifting object up by 0.3m");

    // Get current pose (should be at grasp position)
    geometry_msgs::Pose lift_pose = grasp_pose;

    // Move up 0.3m in Z direction
    lift_pose.position.z += 0.3;

    ROS_INFO("Lifting to position at z=%.3f", lift_pose.position.z);

    // Create waypoints for Cartesian lift
    std::vector<geometry_msgs::Pose> lift_waypoints;
    lift_waypoints.push_back(grasp_pose); // Start position
    lift_waypoints.push_back(lift_pose);  // End position (0.3m up)

    // Plan Cartesian path for lifting (explicit jump threshold + collision avoidance)
    moveit_msgs::RobotTrajectory lift_trajectory;
    const double lift_eef_step = 0.005;
    const bool lift_avoid_collisions = true;

    arm_group->stop();  
    arm_group->setStartStateToCurrentState();
    ros::Duration(0.05).sleep(); 
  
    moveit_msgs::MoveItErrorCodes lift_err;
    double lift_fraction = arm_group->computeCartesianPath(
        lift_waypoints, lift_eef_step, lift_trajectory, lift_avoid_collisions, &lift_err);
    if (lift_err.val != moveit_msgs::MoveItErrorCodes::SUCCESS && lift_fraction >= 0.65)
    {
      ROS_WARN("computeCartesianPath(lift) non-success code %d but acceptable fraction=%.2f",
               lift_err.val, lift_fraction);
    }
    if (lift_fraction < 0.65) // If less than 65% of path achieved
    {
      ROS_ERROR("Could not plan full Cartesian path for lifting!");
      res.success = false;
      reset_flag();
      return true;
    }

    // Time-parameterize before execution
    robot_trajectory::RobotTrajectory lift_rt(
        arm_group->getCurrentState()->getRobotModel(), arm_group->getName());
    lift_rt.setRobotTrajectoryMsg(*arm_group->getCurrentState(), lift_trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization lift_iptp;
    bool lift_timing_ok = lift_iptp.computeTimeStamps(lift_rt, 0.3 /*vel*/, 0.3 /*acc*/);
    if (!lift_timing_ok)
    {
      ROS_WARN("Time-parameterization (lift) failed; proceeding may be unstable");
    }
    lift_rt.getRobotTrajectoryMsg(lift_trajectory);

    // Execute the lift
    moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
    lift_plan.trajectory_ = lift_trajectory;

    moveit::core::MoveItErrorCode lift_result = arm_group->execute(lift_plan);

    if (lift_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to execute lift!");
      res.success = false;
      reset_flag();
      return true;
    }
    arm_group->clearPoseTargets();

    ROS_INFO("Successfully lifted object!");

    ROS_INFO("Pick sequence completed successfully!");
    res.success = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in pickObjectCallback: %s", e.what());
    reset_flag();
    res.success = false;
  }

  return true;
}

bool placeObjectCallback(assignment2_package::PlaceObject::Request &req,
                         assignment2_package::PlaceObject::Response &res)
{
  ROS_INFO("PlaceObject service called - executing place sequence with attached object");
  try
  {
    int id = req.target_id;

    // REMOVED STEP 0 - We keep the object attached in MoveIt during planning
    // This ensures collision checking considers the attached object

    ROS_INFO("STEP 1: Moving down with object attached in both MoveIt and Gazebo");

    // Get current pose with delay for stability
    ros::Duration(0.5).sleep();
    geometry_msgs::PoseStamped current_pose_stamped = arm_group->getCurrentPose();
    geometry_msgs::Pose current_pose = current_pose_stamped.pose;

    // Calculate target pose with buffer
    geometry_msgs::Pose target_pose = current_pose;
    double placement_z_with_buffer = saved_pickup_z;
    target_pose.position.z = placement_z_with_buffer;

    ROS_INFO("Moving down from z=%.3f to z=%.3f (with attached object, includes 0.050m buffer)",
             current_pose.position.z, target_pose.position.z);

    // REMOVED: Force-removing collision object - we want to keep it for collision checking

    // ========== JOINT-SPACE PLANNING WITH ATTACHED OBJECT ==========
    ROS_INFO("STEP 2: Using joint-space planning for placement movement (with attached object)");

    // Set tolerances for more reliable planning
    arm_group->setGoalPositionTolerance(0.010);   // 10 mm
    arm_group->setGoalOrientationTolerance(0.03); // ~1.7°
    arm_group->setPlanningTime(2.0);
    arm_group->setNumPlanningAttempts(20);

    // Set the target pose
    arm_group->setPoseTarget(target_pose);

    // Slow down the movement for safety with attached object
    arm_group->setMaxVelocityScalingFactor(0.2);
    arm_group->setMaxAccelerationScalingFactor(0.2);

    // Plan the movement (MoveIt will consider the attached object for collision checking)
    moveit::planning_interface::MoveGroupInterface::Plan place_plan;
    arm_group->setStartStateToCurrentState();
    bool plan_success = (arm_group->plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!plan_success)
    {
      ROS_WARN("First planning attempt failed, trying with increased tolerances...");

      // Try with more relaxed tolerances
      arm_group->setGoalPositionTolerance(0.02);
      arm_group->setGoalOrientationTolerance(0.02);
      arm_group->setPlanningTime(15.0);

      plan_success = (arm_group->plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!plan_success)
      {
        ROS_ERROR("Failed to plan placement movement even with relaxed tolerances");

        // Clean up: detach the object from MoveIt if planning failed
        moveit_msgs::AttachedCollisionObject detach_cleanup;
        detach_cleanup.link_name = MOVEIT_ATTACH_LINK;
        detach_cleanup.object.id = attached_object_id;
        detach_cleanup.object.operation = moveit_msgs::CollisionObject::REMOVE;
        detach_cleanup.object.header.frame_id = MOVEIT_ATTACH_LINK;
        planning_scene_interface->applyAttachedCollisionObject(detach_cleanup);

        res.success = false;
        is_object_attached = false;
        attached_object_id = "";
        return true;
      }
    }

    ROS_INFO("Successfully planned placement movement with attached object collision checking");

    // Execute the movement
    ROS_INFO("Executing placement movement with reduced speed...");
    moveit::core::MoveItErrorCode place_result = arm_group->execute(place_plan);
    arm_group->clearPoseTargets();
    // Reset scaling factors for future movements
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);

    if (place_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Execution returned error code %d, but continuing with release...", place_result.val);
      // Continue anyway as the robot might be close enough to the target
    }
    else
    {
      ROS_INFO("Successfully executed placement movement");
    }

    ROS_INFO("Reached placement position");

    // Wait for arm to fully settle
    ros::Duration(1.0).sleep();

    // NOW we can detach from MoveIt planning scene (after movement is complete)
    ROS_INFO("Detaching object from MoveIt planning scene after successful placement");
    moveit_msgs::AttachedCollisionObject detach_from_moveit;
    detach_from_moveit.link_name = MOVEIT_ATTACH_LINK;
    detach_from_moveit.object.id = attached_object_id;
    detach_from_moveit.object.operation = moveit_msgs::CollisionObject::REMOVE;
    detach_from_moveit.object.header.frame_id = MOVEIT_ATTACH_LINK;
    planning_scene_interface->applyAttachedCollisionObject(detach_from_moveit);
    ros::Duration(0.5).sleep(); // Allow planning scene to update

    // STEP 3: Detach from Gazebo physics
    ROS_INFO("STEP 3: Detaching object from gripper in Gazebo physics");
    std::string gazebo_detach_service = "/link_attacher_node/detach";
    bool gazebo_detach_service_exists = ros::service::waitForService(
        gazebo_detach_service, ros::Duration(2.0));

    if (gazebo_detach_service_exists)
    {
      ros::ServiceClient detach_client = nh_ptr->serviceClient<gazebo_ros_link_attacher::Attach>(
          gazebo_detach_service);
      gazebo_ros_link_attacher::Attach detach_srv;

      detach_srv.request.model_name_1 = "tiago";
      detach_srv.request.link_name_1 = GRIPPER_LINK_NAME;

      std::string object_model_name;
      std::string object_link_name;

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

      if (detach_client.call(detach_srv))
      {
        if (detach_srv.response.ok)
        {
          ROS_INFO("Successfully detached object %s from gripper in Gazebo",
                   object_model_name.c_str());
        }
        else
        {
          ROS_WARN("Failed to detach object in Gazebo");
        }
      }
      else
      {
        ROS_WARN("Failed to call Gazebo detach service - continuing without detachment");
      }
    }
    else
    {
      ROS_WARN("Gazebo link detacher service not available - continuing without detachment");
    }

    // Wait for physics to settle after Gazebo detachment
    ros::Duration(1.0).sleep();

    // STEP 4: Open the gripper
    ROS_INFO("STEP 4: Opening gripper to release object");

    try
    {
      // Initialize gripper controller
      gripper_control::GripperController gripper("gripper");
      gripper.registerNamedTarget("open", 0.044);

      // Open gripper
      gripper.open();

      // Wait for gripper to open
      ros::Duration(0.5).sleep();

      ROS_INFO("Gripper opened successfully!");
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Failed to open gripper: %s", e.what());
      res.success = false;
      return true;
    }

    // STEP 5: Optional - Move up slightly after releasing
    ROS_INFO("STEP 5: Moving up slightly after release");
    try
    {
      geometry_msgs::Pose retreat_pose = target_pose;
      retreat_pose.position.z += 0.1; // Move up 10cm

      // Use joint-space planning for retreat as well
      arm_group->setPoseTarget(retreat_pose);
      arm_group->setMaxVelocityScalingFactor(0.2); // Faster for retreat
      arm_group->setMaxAccelerationScalingFactor(0.2);

      moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
      bool retreat_success = (arm_group->plan(retreat_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (retreat_success)
      {
        arm_group->execute(retreat_plan);
        arm_group->clearPoseTargets();
        ROS_INFO("Successfully retreated from placement position");
      }
      else
      {
        ROS_WARN("Could not plan retreat, but object was placed successfully");
      }

      // Reset scaling
      arm_group->setMaxVelocityScalingFactor(0.5);
      arm_group->setMaxAccelerationScalingFactor(0.5);
    }
    catch (...)
    {
      ROS_WARN("Exception during retreat, but object was placed successfully");
    }

    // Reset attachment flags
    is_object_attached = false;
    attached_object_id = "";
    saved_pickup_z = 0.0; // Reset for next pick

    ROS_INFO("Place sequence completed successfully!");
    res.success = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Exception in placeObjectCallback: %s", e.what());
    is_object_attached = false;
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
    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setPoseReferenceFrame(BASE_FRAME);
    arm_group->setGoalPositionTolerance(0.010);   // 10 mm
    arm_group->setGoalOrientationTolerance(0.03); // ~1.7°
    arm_group->setPlanningTime(2.0);
    arm_group->setNumPlanningAttempts(20);
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
  ros::ServiceServer place_service = nh.advertiseService("place_object", placeObjectCallback);

  ROS_INFO("Node C: Ready with arm_torso group for extended reach");

  ros::waitForShutdown();

  return 0;
}