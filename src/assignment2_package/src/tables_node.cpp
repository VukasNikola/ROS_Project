#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tables_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // give Gazebo / MoveIt a moment to spin up
    ros::Duration(1.0).sleep();

    moveit::planning_interface::PlanningSceneInterface psi;

    // TF listener to get robot pose
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // common box primitive
    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {1.07, 1.07, 0.77};
    const double z_center = box.dimensions[2] / 2.0;

    ros::Rate rate(10.0); // 10 Hz

    while (ros::ok())
    {
        double robot_x = 0;
        double robot_y = 0;

        // Get robot pose from TF
        try
        {
            geometry_msgs::TransformStamped transform =
                tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
            robot_x = transform.transform.translation.x;
            robot_y = transform.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            rate.sleep();
            continue;
        }

        // Calculate delta
        double delta_x = robot_x - 8.88;

        // Picking table
        moveit_msgs::CollisionObject pick_table;
        pick_table.id = "picking_table";
        pick_table.header.frame_id = "map";
        pick_table.header.stamp = ros::Time::now();
        pick_table.primitives.push_back(box);
        {
            geometry_msgs::Pose p;
            p.position.x = 7.8 + delta_x;
            p.position.y = -3.02;
            p.position.z = z_center;
            p.orientation.w = 1.0;
            pick_table.primitive_poses.push_back(p);
        }

        // Placing table
        moveit_msgs::CollisionObject place_table;
        place_table.id = "placing_table";
        place_table.header.frame_id = "map";
        place_table.header.stamp = ros::Time::now();
        place_table.primitives.push_back(box);
        {
            geometry_msgs::Pose p;
            p.position.x = 7.8 + delta_x;
            p.position.y = -1.92;
            p.position.z = z_center;
            p.orientation.w = 1.0;
            place_table.primitive_poses.push_back(p);
        }

        // bundle both
        std::vector<moveit_msgs::CollisionObject> objects = {pick_table, place_table};

        // refresh both tables
        psi.applyCollisionObjects(objects);

        rate.sleep();
    }

    return 0;
}