// gripper_control.h
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <map>
#include <string>
#include <stdexcept>
#include <cmath>

namespace gripper_control {

// Maximum opening per finger (m)
static constexpr double MAX_FINGER_OPENING = 0.044;

// --- Helper: map AprilTag ID to desired per-finger width ---
// Returns the half-width per finger, clamped to MAX_FINGER_OPENING.
inline double getWidthForTag(int tag_id) {
    double full_width;
    switch (tag_id) {
        case 1: case 2: case 3:
            full_width = std::sqrt(3.0) * 0.05;  // hex prism
            break;
        case 4: case 5: case 6:
            full_width = 0.05;                   // cube
            break;
        case 7: case 8: case 9:
            full_width = 0.07;                   // triangular prism
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
      : gripper_group_(planning_group)
    {}

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

    // Plan, execute, and finalize motion to a given per-finger width
    void setWidth(double width) {
        double w = std::min(std::max(width, 0.0), MAX_FINGER_OPENING);
        std::map<std::string, double> joint_target{
            {"gripper_left_finger_joint",  w},
            {"gripper_right_finger_joint", w}
        };
        gripper_group_.setJointValueTarget(joint_target);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (gripper_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!ok)
            throw std::runtime_error("Gripper planning failed for width: " + std::to_string(w));

        // Execute
        gripper_group_.execute(plan);

        // Ensure no residual commands
        gripper_group_.stop();
        gripper_group_.clearPoseTargets();
    }

    void open()  { 
    // Directly plan & execute open motion
    auto it = named_targets_.find("open");
    if (it == named_targets_.end()) throw std::runtime_error("Open target not registered");
    setWidth(it->second);
}
    void close() { setTarget("closed"); }
    void close(int tag_id) { setWidth(getWidthForTag(tag_id)); }

private:
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    std::map<std::string, double> named_targets_;
};

} // namespace gripper_control