#include "ros/ros.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <boost/bind.hpp>
#include <string>

tf2_ros::TransformBroadcaster *brPtr;
tf2_ros::StaticTransformBroadcaster *staticBrPtr;
tf2_ros::Buffer *tfBufferPtr;
bool mapPublished = false;

// Invert a TransformStamped
geometry_msgs::TransformStamped invertTransform(const geometry_msgs::TransformStamped &input)
{
    tf2::Transform tf;
    tf2::fromMsg(input.transform, tf);
    tf2::Transform inv = tf.inverse();

    geometry_msgs::TransformStamped output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = input.child_frame_id;
    output.child_frame_id = input.header.frame_id;
    output.transform = tf2::toMsg(inv);
    return output;
}

void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (mapPublished)
        return;

    for (const auto &tag : msg->detections)
    {
        if (tag.id[0] != 1)
            continue; // Only tag_1 is relevant

        try
        {
            // Look up map -> tag_1
            geometry_msgs::TransformStamped map_to_tag =
                tfBufferPtr->lookupTransform("map", "tag_1", ros::Time(0), ros::Duration(1.0));

            // Invert to get tag_1 -> map
            geometry_msgs::TransformStamped tag_to_map = invertTransform(map_to_tag);
            tag_to_map.header.stamp = ros::Time::now();
            tag_to_map.header.frame_id = "tag_1";
            tag_to_map.child_frame_id = "map";

            // Broadcast static transform
            staticBrPtr->sendTransform(tag_to_map);
            ROS_INFO("Published static transform: tag_1 -> map");

            mapPublished = true; // Only publish once
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not transform map -> tag_1: %s", ex.what());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_to_map_broadcaster");
    ros::NodeHandle nh;

    static tf2_ros::TransformBroadcaster br;
    static tf2_ros::StaticTransformBroadcaster staticBr;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    brPtr = &br;
    staticBrPtr = &staticBr;
    tfBufferPtr = &tfBuffer;

    message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> sub(nh, "tag_detections", 10);
    tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray> *mf;
    mf = new tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray>(sub, tfBuffer, "map", 10, nh);
    mf->registerCallback(boost::bind(&detectionCallback, _1));

    ros::spin();
    return 0;
}
