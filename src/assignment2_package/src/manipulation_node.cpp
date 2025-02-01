#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <assignment2_package/PickObject.h>    // Custom service: empty request; response: bool success, string message
#include <assignment2_package/PlaceObject.h>   // Custom service: request: geometry_msgs/PoseStamped target_pose; response: bool success, string message
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <vector>

// Global persistent NodeHandle pointer.
ros::NodeHandle* g_nh = nullptr;

// Global variable to store the candidate object pose.
geometry_msgs::PoseStamped g_object_pose;
bool g_object_pose_received = false;

void objectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  g_object_pose = *msg;
  g_object_pose_received = true;
  ROS_INFO("Received object pose for picking.");
}

// Gripper control function using a persistent NodeHandle.
bool controlGripper(ros::NodeHandle &nh, const std::string &command)
{
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/gripper_controller/" + command);
  std_srvs::Trigger srv;
  if(client.waitForExistence(ros::Duration(5.0))) {
    if(client.call(srv)) {
      ROS_INFO("Gripper %s: %s", command.c_str(), srv.response.message.c_str());
      return srv.response.success;
    } else {
      ROS_WARN("Failed to call gripper %s service", command.c_str());
    }
  } else {
    ROS_WARN("Gripper %s service not available", command.c_str());
  }
  return false;
}

bool pickObjectCallback(assignment2_package::PickObject::Request &req,
                        assignment2_package::PickObject::Response &res)
{
  if (!g_object_pose_received) {
    res.success = false;
    res.message = "No object pose received.";
    return true;
  }

  moveit::planning_interface::MoveGroupInterface move_group("arm");
  move_group.setPlanningTime(10.0);

  // Step 5: Move to pre-grasp pose (10 cm above the object).
  geometry_msgs::Pose pre_grasp_pose = g_object_pose.pose;
  pre_grasp_pose.position.z += 0.10;
  move_group.setPoseTarget(pre_grasp_pose);
  ROS_INFO("Planning to pre-grasp pose...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan pre-grasp pose.";
    return true;
  }
  move_group.execute(plan);

  // Step 6: Linear approach – lower by 8 cm.
  geometry_msgs::Pose grasp_pose = pre_grasp_pose;
  grasp_pose.position.z -= 0.08;
  move_group.setPoseTarget(grasp_pose);
  ROS_INFO("Planning grasp approach...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan grasp approach.";
    return true;
  }
  move_group.execute(plan);

  // Step 8: Attach object virtually (simulation).
  ROS_INFO("Attaching object (simulation)...");

  // Step 9: Close the gripper.
  ROS_INFO("Closing gripper...");
  controlGripper(*g_nh, "close");
  ros::Duration(1.0).sleep();

  // Step 10: Retract to pre-grasp pose.
  move_group.setPoseTarget(pre_grasp_pose);
  ROS_INFO("Planning retreat...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan retreat.";
    return true;
  }
  move_group.execute(plan);

  // Steps 11 & 12: Move to an intermediate (secure) pose.
  std::vector<double> joint_group_positions;
  move_group.getCurrentState()->copyJointGroupPositions(
    move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
    joint_group_positions);
  if(joint_group_positions.size() >= 3) {
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = -1.0;
    joint_group_positions[2] = 1.0;
  }
  move_group.setJointValueTarget(joint_group_positions);
  ROS_INFO("Moving to secure pose...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan secure pose.";
    return true;
  }
  move_group.execute(plan);

  res.success = true;
  res.message = "Pick operation completed.";
  return true;
}

bool placeObjectCallback(assignment2_package::PlaceObject::Request &req,
                         assignment2_package::PlaceObject::Response &res)
{
  moveit::planning_interface::MoveGroupInterface move_group("arm");
  move_group.setPlanningTime(10.0);
  
  // Step 14: Move to pre-place pose (10 cm above target).
  geometry_msgs::Pose target_pose = req.target_pose.pose;
  geometry_msgs::Pose pre_place_pose = target_pose;
  pre_place_pose.position.z += 0.10;
  move_group.setPoseTarget(pre_place_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ROS_INFO("Planning to pre-place pose...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan pre-place pose.";
    return true;
  }
  move_group.execute(plan);
  
  // Lower for placement.
  move_group.setPoseTarget(target_pose);
  ROS_INFO("Planning place motion...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan place motion.";
    return true;
  }
  move_group.execute(plan);
  
  // Step 15: Open the gripper.
  ROS_INFO("Opening gripper...");
  controlGripper(*g_nh, "open");
  ros::Duration(1.0).sleep();
  
  // Step 16: Detach object (simulation).
  ROS_INFO("Detaching object (simulation)...");
  
  // Retract to a safe pose.
  pre_place_pose.position.z += 0.10;
  move_group.setPoseTarget(pre_place_pose);
  ROS_INFO("Planning retreat after placing...");
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    res.success = false;
    res.message = "Failed to plan retreat after placing.";
    return true;
  }
  move_group.execute(plan);
  
  res.success = true;
  res.message = "Place operation completed.";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_node");
  ros::NodeHandle nh;
  g_nh = &nh;

  ros::Subscriber obj_sub = nh.subscribe("object_pick_pose", 10, objectPoseCallback);
  ros::ServiceServer pick_service = nh.advertiseService("pick_object", pickObjectCallback);
  ros::ServiceServer place_service = nh.advertiseService("place_object", placeObjectCallback);

  ROS_INFO("Manipulation node ready. Waiting for pick and place commands.");
  ros::spin();
  return 0;
}
