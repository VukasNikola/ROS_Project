// ids_generator_node.cpp

#include "ros/ros.h"
#include "apriltag_finder/ApriltagsIDsSrv.h" // Correct service include
#include <vector>
#include <cstdlib>
#include <ctime>

bool generateIDs(apriltag_finder::ApriltagsIDsSrv::Request &req,
                apriltag_finder::ApriltagsIDsSrv::Response &res)
{
    // For example, generate 5 random IDs between 0 and 100
    int num_ids = 5;
    res.ids.clear();
    for (int i = 0; i < num_ids; ++i)
    {
        res.ids.push_back(rand() % 100);
    }
    ROS_INFO("ids_generator_node: Generated IDs: %s", [&res]() -> std::string {
        std::ostringstream oss;
        for (size_t i = 0; i < res.ids.size(); ++i)
        {
            oss << res.ids[i];
            if (i != res.ids.size() - 1)
                oss << ", ";
        }
        return oss.str();
    }().c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ids_generator_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("apriltags_ids_srv", generateIDs);
    ROS_INFO("ids_generator_node: Ready to generate Apriltag IDs.");
    srand(time(NULL)); // Seed for random number generation

    ros::spin();
    return 0;
}
