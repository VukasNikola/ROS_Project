// gripper_control.h - IMPROVED VERSION
#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <stdexcept>
#include <cmath>

namespace gripper_control {

// Maximum opening per finger (m)
static constexpr double MAX_FINGER_OPENING = 0.044;

// Helper: map AprilTag ID to desired per-finger width
inline double getWidthForTag(int tag_id) {
    double full_width;
    switch (tag_id) {
        case 1: case 2: case 3:
            full_width = 0.05; // hex prism
            break;
        case 4: case 5: case 6:
            full_width = 0.05; // cube
            break;
        case 7: case 8: case 9:
            full_width = 0.068; // triangular prism
            break;
        default:
            throw std::invalid_argument("Unknown AprilTag ID: " + std::to_string(tag_id));
    }
    double per_finger = full_width / 2.0;
    return std::min(per_finger, MAX_FINGER_OPENING);
}

class GripperController {
public:
    explicit GripperController(const std::string& planning_group)
        : gripper_group_(planning_group) {
        // Set reasonable timeouts
        gripper_group_.setPlanningTime(5.0);
        gripper_group_.setMaxVelocityScalingFactor(0.3);
        gripper_group_.setMaxAccelerationScalingFactor(0.3);
    }

    void registerNamedTarget(const std::string& name, double width) {
        double w = std::min(std::max(width, 0.0), MAX_FINGER_OPENING);
        named_targets_[name] = w;
    }

    void setTarget(const std::string& name) {
        auto it = named_targets_.find(name);
        if (it == named_targets_.end())
            throw std::runtime_error("Unknown target: " + name);
        setWidth(it->second);
    }

    // Plan, execute, and wait for completion
    void setWidth(double width) {
        double w = std::min(std::max(width, 0.0), MAX_FINGER_OPENING);
        
        ROS_INFO("Setting gripper width to: %f", w);
        
        std::map<std::string, double> joint_target{
            {"gripper_left_finger_joint", w},
            {"gripper_right_finger_joint", w}
        };
        
        gripper_group_.setJointValueTarget(joint_target);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto planning_result = gripper_group_.plan(plan);
        
        if (planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Gripper planning failed for width");
        }

        // Execute and wait for completion
        auto execution_result = gripper_group_.execute(plan);
        if (execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Gripper execution failed");
        }

        // Wait for movement to complete
        ros::Duration(1.0).sleep();
        
        // Verify final position (optional but helpful for debugging)
        auto current_joints = gripper_group_.getCurrentJointValues();
        if (current_joints.size() >= 2) {
            ROS_INFO("Gripper final positions - Left: %f, Right: %f", 
                     current_joints[0], current_joints[1]);
        }
    }

    void open() {
        auto it = named_targets_.find("open");
        if (it == named_targets_.end()) 
            throw std::runtime_error("Open target not registered");
        
        ROS_INFO("Opening gripper...");
        setWidth(it->second);
    }

    void close() { 
        ROS_INFO("Closing gripper to closed position...");
        setTarget("closed"); 
    }

    void close(int tag_id) { 
        ROS_INFO("Closing gripper for tag %d...", tag_id);
        setWidth(getWidthForTag(tag_id)); 
    }

    // Get current gripper state for debugging
    std::vector<double> getCurrentJointValues() {
        return gripper_group_.getCurrentJointValues();
    }

private:
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    std::map<std::string, double> named_targets_;
};

} // namespace gripper_control