// cmd_vel_subscriber.cpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Publisher to forward cmd_vel messages to Tiago's movement controller
ros::Publisher tiago_cmd_vel_pub;

// Callback function to handle incoming /cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Log the received velocity commands
    ROS_INFO("Received /cmd_vel - Linear X: %f, Angular Z: %f", msg->linear.x, msg->angular.z);

    // Forward the received Twist message to Tiago's movement controller
    tiago_cmd_vel_pub.publish(*msg);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "cmd_vel_subscriber");
    ros::NodeHandle nh;

    // Initialize the publisher to Tiago's cmd_vel topic
    // Replace '/tiago/cmd_vel' with the actual topic Tiago's controller listens to
    tiago_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tiago/cmd_vel", 10);

    // Subscribe to the standard /cmd_vel topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    ROS_INFO("cmd_vel_subscriber node started. Forwarding /cmd_vel to /tiago/cmd_vel...");

    // Spin to keep the node active and responsive to callbacks
    ros::spin();

    return 0;
}
