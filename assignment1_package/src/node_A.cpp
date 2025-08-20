// Updated Node A - Action Client
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <assignment1_package/NavigationTaskAction.h>
#include <vector>

typedef actionlib::SimpleActionClient<assignment1_package::NavigationTaskAction> NavigationClient;

class NodeA {
private:
    ros::NodeHandle nh_;
    NavigationClient navigation_client_;
    std::vector<int> target_ids_;
    
public:
    NodeA() : navigation_client_("navigation_task", true) {
        ROS_INFO("[Node A] Waiting for navigation action server...");
        navigation_client_.waitForServer();
        ROS_INFO("[Node A] Connected to navigation action server!");
    }
    
    // Feedback callback - prints robot status
    void feedbackCallback(const assignment1_package::NavigationTaskFeedbackConstPtr& feedback) {
        ROS_INFO("[Node A] Robot Status: %s", feedback->status.c_str());
        
        if (!feedback->found_tag_ids.empty()) {
            std::ostringstream oss;
            oss << "[Node A] Found AprilTags with IDs: ";
            for (int id : feedback->found_tag_ids) {
                oss << id << " ";
            }
            ROS_INFO_STREAM(oss.str());
        }
    }
    
    // Result callback - receives final cube positions
    void resultCallback(const actionlib::SimpleClientGoalState& state,
                       const assignment1_package::NavigationTaskResultConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("[Node A] Navigation task completed successfully!");
            ROS_INFO("[Node A] Received %zu cube positions:", result->cube_positions.poses.size());
            
            for (size_t i = 0; i < result->cube_positions.poses.size(); ++i) {
                const auto& pose = result->cube_positions.poses[i];
                ROS_INFO("[Node A] Cube %zu: [%.2f, %.2f, %.2f]", i+1,
                         pose.position.x, pose.position.y, pose.position.z);
            }
        } else {
            ROS_WARN("[Node A] Navigation task failed with state: %s", state.toString().c_str());
        }
    }
    
    void run() {
        // Get target IDs from AprilTag service
        ros::ServiceClient client = nh_.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");
        
        ROS_INFO("[Node A] Waiting for service /apriltag_ids_srv...");
        client.waitForExistence();
        
        tiago_iaslab_simulation::Objs srv;
        srv.request.ready = true;
        
        if (client.call(srv)) {
            target_ids_ = srv.response.ids;
            
            std::ostringstream oss;
            oss << "[Node A] Received AprilTag IDs: ";
            for (int id : target_ids_) {
                oss << id << " ";
            }
            ROS_INFO_STREAM(oss.str());
            
            // Send navigation goal to Node B
            assignment1_package::NavigationTaskGoal goal;
            goal.target_ids = target_ids_;
            
            ROS_INFO("[Node A] Sending navigation task to Node B...");
            navigation_client_.sendGoal(goal,
                boost::bind(&NodeA::resultCallback, this, _1, _2),
                NavigationClient::SimpleActiveCallback(),
                boost::bind(&NodeA::feedbackCallback, this, _1));
            
            // Wait for completion
            navigation_client_.waitForResult();
            
        } else {
            ROS_ERROR("[Node A] Failed to call service /apriltag_ids_srv");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    
    NodeA node_a;
    node_a.run();
    
    ros::spin();
    return 0;
}
