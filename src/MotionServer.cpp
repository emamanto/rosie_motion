#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include "rosie_msgs/RobotCommand.h"
#include "rosie_msgs/Observations.h"

void obsCallback(const rosie_msgs::Observations::ConstPtr& msg)
{
  ROS_INFO("I heard a message");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosie_motion_server");
  ros::NodeHandle n;
  ros::Rate loopRate(1);

  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("EE frame: %s", group.getEndEffectorLink().c_str());

  ros::Subscriber sub = n.subscribe("rosie_observations", 1000, obsCallback);

  while (ros::ok()) {
    ROS_INFO("Node is running");
    ros::spinOnce();
    loopRate.sleep();
  }

}
