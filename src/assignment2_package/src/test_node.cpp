#include <ros/ros.h>
#include <assignment2_package/GetObjectPose.h>
#include <assignment2_package/PickObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  // Client for GetObjectPose
  ros::ServiceClient get_pose_client = nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
  ROS_INFO("Waiting for get_object_pose service...");
  get_pose_client.waitForExistence();

  // Client for PickObject
  ros::ServiceClient pick_client = nh.serviceClient<assignment2_package::PickObject>("pick_object");
  ROS_INFO("Waiting for pick_object service...");
  pick_client.waitForExistence();

  // 1) Call get_object_pose
  assignment2_package::GetObjectPose get_pose_srv;
  if (!get_pose_client.call(get_pose_srv)) {
    ROS_ERROR("Failed to call get_object_pose service");
    return 1;
  }
  // Optionally check inside the response that an object was found:
  // if (!get_pose_srv.response.success) { ... }

  // 2) Populate and call pick_object
  assignment2_package::PickObject pick_srv;
  pick_srv.request.target   = get_pose_srv.response.obj_pose;
  pick_srv.request.target_id = get_pose_srv.response.obj_id;

  if (pick_client.call(pick_srv)) {
    if (pick_srv.response.success) {
      ROS_INFO("Pick succeeded!");
    } else {
      ROS_WARN("Pick service returned failure");
    }
  } else {
    ROS_ERROR("Failed to call pick_object service");
    return 1;
  }

  return 0;
}
