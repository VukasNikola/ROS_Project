#ifndef TIAGO_IASLAB_SIMULATION_APRILTAGIDS_H
#define TIAGO_IASLAB_SIMULATION_APRILTAGIDS_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <algorithm>
#include <vector>

class ApriltagIds{
    public:
        ApriltagIds(std::shared_ptr<ros::NodeHandle> nh_ptr);
        ~ApriltagIds() {}

    private:

        std::shared_ptr<ros::NodeHandle> nh_ptr_;
        ros::ServiceServer objects_server_;

        void start();
        bool objService(tiago_iaslab_simulation::Objs::Request &req, tiago_iaslab_simulation::Objs::Response &res);
        std::vector<int> generate_ids();
};

#endif
