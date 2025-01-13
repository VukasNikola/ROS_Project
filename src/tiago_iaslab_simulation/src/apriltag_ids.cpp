#include <tiago_iaslab_simulation/ApriltagIds.h>
#include <random>

ApriltagIds::ApriltagIds(std::shared_ptr<ros::NodeHandle> nh_ptr)
: nh_ptr_(nh_ptr)
{
  start();
}

void ApriltagIds::start(){
  objects_server_ = nh_ptr_->advertiseService("/apriltag_ids_srv", &ApriltagIds::objService, this);
  ROS_INFO_STREAM("Service called!");
}

std::vector<int> ApriltagIds::generate_ids()
{
    std::vector<int> ids;
    for(int i = 1; i<=15; i++){
        ids.push_back(i);
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(ids.begin(), ids.end(), g);
    std::vector<int> sequence(ids.begin(), ids.begin() + 5);

    return sequence;
} 

bool ApriltagIds::objService(tiago_iaslab_simulation::Objs::Request &req, tiago_iaslab_simulation::Objs::Response &res){
    if(!req.ready){
        ROS_ERROR_STREAM("Error! Ready is false. It must be TRUE");
        ros::shutdown();
        return false;
    }
    std::vector<int> ids_vec;
    ids_vec = ApriltagIds::generate_ids();
    res.ids = ids_vec;
    return true;
}
