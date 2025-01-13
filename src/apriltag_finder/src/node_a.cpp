// node_a.cpp

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "apriltag_finder/ApriltagsIDsSrv.h" // Correct service include
#include <sstream>
#include <vector>
#include <string>

class NodeA
{
public:
    NodeA()
    {
        // Initialize publishers and subscribers
        ids_pub_ = nh_.advertise<std_msgs::String>("/selected_apriltag_ids", 10);
        feedback_sub_ = nh_.subscribe("/node_b/feedback", 10, &NodeA::feedbackCallback, this);
        cube_pos_sub_ = nh_.subscribe("/node_b/cube_positions", 10, &NodeA::cubePositionsCallback, this);

        // Initialize service client
        apriltags_ids_client_ = nh_.serviceClient<apriltag_finder::ApriltagsIDsSrv>("/apriltags_ids_srv");

        // Wait for the service to be available
        ROS_INFO("Node A: Waiting for /apriltags_ids_srv service...");
        apriltags_ids_client_.waitForExistence();
        ROS_INFO("Node A: Service available. Calling service...");

        // Call the service
        apriltag_finder::ApriltagsIDsSrv srv;
        if (apriltags_ids_client_.call(srv))
        {
            selected_ids_ = srv.response.ids;
            ROS_INFO("Node A: Received IDs: %s", vectorToString(selected_ids_).c_str());

            // Publish the selected IDs to Node B
            std_msgs::String ids_msg;
            ids_msg.data = vectorToString(selected_ids_, ",");
            ids_pub_.publish(ids_msg);
            ROS_INFO("Node A: Published selected IDs to Node B.");
        }
        else
        {
            ROS_ERROR("Node A: Failed to call /apriltags_ids_srv service.");
        }
    }

    void feedbackCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Feedback from Node B: %s", msg->data.c_str());
    }

    void cubePositionsCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        ROS_INFO("Received cube position: [x: %f, y: %f, z: %f]", msg->position.x, msg->position.y, msg->position.z);
        // You can store the positions if needed
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher ids_pub_;
    ros::Subscriber feedback_sub_;
    ros::Subscriber cube_pos_sub_;
    ros::ServiceClient apriltags_ids_client_;
    std::vector<int> selected_ids_;

    std::string vectorToString(const std::vector<int>& vec, const std::string& delimiter = " ")
    {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i != vec.size() - 1)
                oss << delimiter;
        }
        return oss.str();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_a");
    NodeA node_a;
    ros::spin();
    return 0;
}
