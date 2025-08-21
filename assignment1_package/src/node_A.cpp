/**
 * Node A - Action Client
 * 
 * This node serves as the main coordinator for the navigation task.
 * It retrieves target AprilTag IDs from a service and delegates the 
 * navigation and detection task to Node B via ROS actions.
 * 
 * Workflow:
 * 1. Connect to the navigation action server (Node B)
 * 2. Request target AprilTag IDs from the apriltag_ids service
 * 3. Send navigation goal to Node B with the target IDs
 * 4. Monitor progress via feedback callbacks
 * 5. Receive final results with cube positions
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <assignment1_package/NavigationTaskAction.h>
#include <vector>

// Type alias for cleaner code - creates a client for NavigationTask actions
typedef actionlib::SimpleActionClient<assignment1_package::NavigationTaskAction> NavigationClient;

/**
 * NodeA Class
 * 
 * Main action client that coordinates the overall navigation task.
 * Responsible for getting target IDs and communicating with the navigation server.
 */
class NodeA
{
private:
    ros::NodeHandle nh_;                    // ROS node handle for communication
    NavigationClient navigation_client_;    // Action client to communicate with Node B
    std::vector<int> target_ids_;          // List of AprilTag IDs to search for

public:
    /**
     * Constructor
     * Initializes the action client and waits for the navigation server to become available
     */
    NodeA() : navigation_client_("navigation_task", true)
    {
        ROS_INFO("[Node A] Waiting for navigation action server...");
        navigation_client_.waitForServer();
        ROS_INFO("[Node A] Connected to navigation action server!");
    }

    /**
     * Feedback Callback
     * Called periodically during navigation to receive status updates from Node B
     * 
     * @param feedback Feedback message containing current status and found tag IDs
     */
    void feedbackCallback(const assignment1_package::NavigationTaskFeedbackConstPtr &feedback)
    {
        // Print current robot status (e.g., "Moving to waypoint 3", "Scanning...")
        ROS_INFO("[Node A] Robot Status: %s", feedback->status.c_str());
        
        // If any AprilTags have been found, display their IDs
        if (!feedback->found_tag_ids.empty())
        {
            std::ostringstream oss;
            oss << "[Node A] Found AprilTags with IDs: ";
            for (int id : feedback->found_tag_ids)
            {
                oss << id << " ";
            }
            ROS_INFO_STREAM(oss.str());
        }
    }

    /**
     * Result Callback
     * Called when the navigation task completes (successfully or with failure)
     * Receives the final results with all detected cube positions
     * 
     * @param state Final state of the action (SUCCEEDED, FAILED, etc.)
     * @param result Result message containing cube IDs and their 3D positions
     */
    void resultCallback(const actionlib::SimpleClientGoalState &state,
                       const assignment1_package::NavigationTaskResultConstPtr &result)
    {
        // Check if the action completed successfully
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Navigation completed successfully! Found %zu cubes:", result->cube_ids.size());
            
            // Display each found cube with its ID and 3D position in the map frame
            for (size_t i = 0; i < result->cube_ids.size(); ++i)
            {
                int32_t cube_id = result->cube_ids[i];
                const auto &pose = result->cube_positions.poses[i];
                ROS_INFO("Cube ID %d at [%.2f, %.2f, %.2f]",
                         cube_id, pose.position.x, pose.position.y, pose.position.z);
            }
        }
        else
        {
            // Handle different failure cases
            ROS_ERROR("Navigation task failed with state: %s", state.toString().c_str());
            
            if (state == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                ROS_WARN("Navigation was preempted (cancelled)");
            }
            else if (state == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_WARN("Navigation was aborted by the server");
            }
            else
            {
                ROS_WARN("Navigation failed for unknown reason");
            }
        }
    }

    /**
     * Main Execution Function
     * Orchestrates the complete workflow:
     * 1. Get target AprilTag IDs from service
     * 2. Send navigation goal to Node B
     * 3. Wait for completion
     */
    void run()
    {
        // Step 1: Request target AprilTag IDs from the simulation service
        ros::ServiceClient client = nh_.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");
        ROS_INFO("[Node A] Waiting for service /apriltag_ids_srv...");
        client.waitForExistence();

        // Prepare service request
        tiago_iaslab_simulation::Objs srv;
        srv.request.ready = true;  // Signal that we're ready to receive IDs

        if (client.call(srv))
        {
            // Successfully received target IDs
            target_ids_ = srv.response.ids;
            std::ostringstream oss;
            oss << "[Node A] Received AprilTag IDs: ";
            for (int id : target_ids_)
            {
                oss << id << " ";
            }
            ROS_INFO_STREAM(oss.str());

            // Step 2: Send navigation goal to Node B with the target IDs
            assignment1_package::NavigationTaskGoal goal;
            goal.target_ids = target_ids_;

            ROS_INFO("[Node A] Sending navigation task to Node B...");
            
            // Send goal with callbacks:
            // - resultCallback: called when task completes
            // - Empty active callback: called when goal becomes active
            // - feedbackCallback: called periodically during execution
            navigation_client_.sendGoal(goal,
                                       boost::bind(&NodeA::resultCallback, this, _1, _2),
                                       NavigationClient::SimpleActiveCallback(),
                                       boost::bind(&NodeA::feedbackCallback, this, _1));

            // Step 3: Wait for the navigation task to complete
            navigation_client_.waitForResult();
        }
        else
        {
            ROS_ERROR("[Node A] Failed to call service /apriltag_ids_srv");
        }
    }
};

/**
 * Main Function
 * Entry point for Node A
 */
int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "node_a");
    
    // Create and run the NodeA instance
    NodeA node_a;
    node_a.run();
    
    // Keep the node alive to process callbacks
    ros::spin();
    return 0;
}