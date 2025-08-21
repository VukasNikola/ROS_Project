#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class TiagoRotateNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    
    // Rotation parameters
    double angular_velocity_;
    double target_angle_;

public:
    TiagoRotateNode() : angular_velocity_(0.5), target_angle_(2 * M_PI)
    {
        // Create publisher for velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        
        // Wait for publisher to connect
        ros::Duration(1.0).sleep();
        
        ROS_INFO("TIAGo rotation node initialized");
    }
    
    void rotateFullCircle()
    {
        ROS_INFO("Starting full circle rotation...");
        
        // Calculate rotation time needed
        double rotation_time = target_angle_ / angular_velocity_;
        
        // Create Twist message
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_velocity_;  // Positive for counter-clockwise
        
        // Record start time
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10);  // 10 Hz
        
        // Publish rotation commands
        while (ros::ok())
        {
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - start_time).toSec();
            
            if (elapsed_time >= rotation_time)
            {
                break;
            }
            
            cmd_vel_pub_.publish(twist_msg);
            rate.sleep();
        }
        
        // Stop the robot
        stopRobot();
        ROS_INFO("Full circle rotation completed!");
    }
    
    void stopRobot()
    {
        geometry_msgs::Twist stop_msg;
        // All velocities are 0 by default
        cmd_vel_pub_.publish(stop_msg);
        ROS_INFO("Robot stopped");
    }
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "tiago_rotate_node");
    
    try
    {
        // Create and run the rotation node
        TiagoRotateNode tiago_node;
        tiago_node.rotateFullCircle();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error: %s", e.what());
    }
    
    return 0;
}