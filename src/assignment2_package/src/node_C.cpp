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
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <boost/variant.hpp>
#include <map>
#include <set>

// Node C: MoveIt manipulation and collision object management
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string BASE_FRAME = "base_footprint";


// Globals
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

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
    0.5,   // X-scale
    0.5,   // Y-scale
    0.5   // Z-scale
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
      primitive.dimensions = {0.1, 0.025}; // height, radius
      height_offset = primitive.dimensions[0] / 2.0; // half of cylinder height
    }
    else
    {
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.05, 0.05, 0.05}; // x, y, z
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
    else ++it;
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
    ROS_INFO("Node C: PickObject service called.");

    geometry_msgs::PoseStamped target_pose = req.target;
    geometry_msgs::Pose obj_pose = target_pose.pose;

    // Open gripper first
    gripper_group->setNamedTarget("open");
    gripper_group->move();

    // Add table collision object (only once) - this is the original table, different from pick/place tables
    static bool table_added = false;
    if (!table_added)
    {
        shape_msgs::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::SolidPrimitive::BOX;
        table_shape.dimensions = {1.0, 1.0, 0.05}; // 1m x 1m x 5cm table

        geometry_msgs::Pose table_pose;
        table_pose.position.x = 0.7; // in front of robot
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.72; // table height
        table_pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        addCollisionObject("main_table", table_shape, table_pose, BASE_FRAME);
        table_added = true;
        ROS_INFO("Node C: Added main table collision object.");
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
    if (!success)
    {
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

    if (fraction > 0.8)
    { // Accept if 80% of path is planned
        ROS_INFO("Node C: Executing descent to grasp object...");
        arm_group->execute(cart_traj);
    }
    else
    {
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
    for (const auto &tag_pair : detected_tags)
    {
        geometry_msgs::PoseStamped tag_pose = tag_pair.second;

        // Simple distance check to find which object was picked
        double dist = sqrt(pow(tag_pose.pose.position.x - obj_pose.position.x, 2) +
                           pow(tag_pose.pose.position.y - obj_pose.position.y, 2) +
                           pow(tag_pose.pose.position.z - obj_pose.position.z, 2));

        if (dist < 0.1)
        { // Within 10cm - likely the same object
            objects_to_remove.push_back("tag_object_" + std::to_string(tag_pair.first));
            ROS_INFO("Node C: Removing collision object for picked tag %d", tag_pair.first);
            break;
        }
    }

    if (!objects_to_remove.empty())
    {
        planning_scene_interface->removeCollisionObjects(objects_to_remove);
    }

    // Lift object back up
    ROS_INFO("Node C: Lifting object...");
    arm_group->setPoseTarget(approach_pose.pose, "arm_tool_link");
    if (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_C");
  ros::NodeHandle nh;

  // TF
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  // MoveIt interfaces
  static moveit::planning_interface::MoveGroupInterface arm_mg(PLANNING_GROUP_ARM);
  static moveit::planning_interface::MoveGroupInterface gripper_mg(PLANNING_GROUP_GRIPPER);
  static moveit::planning_interface::PlanningSceneInterface psi;
  arm_group = &arm_mg;
  gripper_group = &gripper_mg;
  planning_scene_interface = &psi;

  arm_group->setPlanningTime(10.0);
  arm_group->setMaxVelocityScalingFactor(0.5);
  arm_group->setMaxAccelerationScalingFactor(0.5);

  // Subscribe & service as before
  ros::Subscriber object_sub = nh.subscribe("object_poses", 1, objectPosesCallback);
  ros::ServiceServer pick_service = nh.advertiseService("pick_object", pickObjectCallback);

  ROS_INFO("Node C: Ready (static tables placed at 10, -2.5 in map frame)");
  ros::spin();

  delete tfListener;
  return 0;
}