#ifndef TIAGO_IASLAB_SIMULATION_GETSTRAIGHTLINE_H
#define TIAGO_IASLAB_SIMULATION_GETSTRAIGHTLINE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include <algorithm>
#include <vector>

class GetStraightline{
    public:
        GetStraightline(std::shared_ptr<ros::NodeHandle> nh_ptr);
        ~GetStraightline() {}

    private:

        std::shared_ptr<ros::NodeHandle> nh_ptr_;
        ros::ServiceServer coefficients_server_;

        void start();
        bool coeffsService(tiago_iaslab_simulation::Coeffs::Request &req, tiago_iaslab_simulation::Coeffs::Response &res);
        std::vector<float> generate_coefficients();
};

#endif
