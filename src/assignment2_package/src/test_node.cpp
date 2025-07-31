// test_get_object_pose.cpp
#include <ros/ros.h>
#include "assignment2_package/GetObjectPose.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_get_object_pose");
    ros::NodeHandle nh;

    // Create a client for the service
    ros::ServiceClient client =
        nh.serviceClient<assignment2_package::GetObjectPose>("get_object_pose");
    assignment2_package::GetObjectPose srv;

    ros::Rate rate(1.0); // 1 Hz
    while (ros::ok())
    {
        if (client.call(srv))
        {
            ROS_INFO("Got object ID %u in frame '%s' at [%.3f, %.3f, %.3f]",
                     srv.response.obj_id,
                     srv.response.obj_pose.header.frame_id.c_str(),
                     srv.response.obj_pose.pose.position.x,
                     srv.response.obj_pose.pose.position.y,
                     srv.response.obj_pose.pose.position.z);
        }

        else
        {
            // Service call failed or returned false
            ROS_WARN("Service call to get_object_pose failed or no object available.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
