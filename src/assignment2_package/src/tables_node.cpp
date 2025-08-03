#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tables_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // give Gazebo / MoveIt a moment to spin up
  ros::Duration(1.0).sleep();

  moveit::planning_interface::PlanningSceneInterface psi;

  // common box primitive
  shape_msgs::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {1, 1, 0.77};

  const double z_center = box.dimensions[2] / 2.0;

  // Picking table
  moveit_msgs::CollisionObject pick_table;
  pick_table.id = "picking_table";
  pick_table.header.frame_id = "map";
  pick_table.primitives.push_back(box);
  {
    geometry_msgs::Pose p;
    p.position.x = 7.8;
    p.position.y = -3.02;
    p.position.z = z_center;
    p.orientation.w = 1.0;
    pick_table.primitive_poses.push_back(p);
  }

  // Placing table
  moveit_msgs::CollisionObject place_table;
  place_table.id = "placing_table";
  place_table.header.frame_id = "map";
  place_table.primitives.push_back(box);
  {
    geometry_msgs::Pose p;
    p.position.x = 7.8;
    p.position.y = -1.92;
    p.position.z = z_center;
    p.orientation.w = 1.0;
    place_table.primitive_poses.push_back(p);
  }

  // bundle both
  std::vector<moveit_msgs::CollisionObject> objects = { pick_table, place_table };

  ros::Rate rate(10.0);  // 10 Hz
  while (ros::ok())
  {
    // update timestamps
    ros::Time now = ros::Time::now();
    for (auto& obj : objects)
      obj.header.stamp = now;

    // refresh both tables
    psi.applyCollisionObjects(objects);

    rate.sleep();
  }

  return 0;
}
