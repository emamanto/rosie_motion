#pragma once

#include <iostream>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <ros/ros.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>

#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "moveit_msgs/ApplyPlanningScene.h"
#include "moveit_msgs/GetPlanningScene.h"

class ArmController {
public:
  typedef std::vector<moveit::planning_interface::MoveGroupInterface::Plan> PlanVector;
  typedef std::pair<tf2::Transform, tf2::Transform> GraspPair;

  ArmController(ros::NodeHandle& nh);
  void setHumanChecks(bool on) { checkPlans = on; }

  std::string armPlanningFrame();
  std::string getHeld() { return grabbedObject.id; }

  void updateCollisionScene(std::vector<moveit_msgs::CollisionObject> cos);
  void attachToGripper(std::string objName);
  void detachHeldObject();

  void closeGripper();
  void openGripper();
  void setGripperClosed(bool isClosed);

  bool pickUp(tf2::Transform objXform,
              std::vector<GraspPair> graspList,
              std::string objName);
  bool putDownHeldObj(std::vector<tf2::Transform> targets);
  bool pointTo(tf2::Transform objXform,
               float objHeight);
  bool homeArm();

private:
  void setGripperTo(float m);
  bool planToXform(tf2::Transform t);
  bool planToXform(tf2::Transform t, int n);
  double planStraightLineMotion(tf2::Transform target);
  bool executeCurrentPlan();
  bool safetyCheck();
  void publishCurrentGoal(const ros::TimerEvent& e);
  void setCurrentGoalTo(tf2::Transform t);

  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  int numRetries;
  moveit_msgs::CollisionObject grabbedObject;
  GraspPair usedGrasp;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
  bool checkPlans;
  bool gripperClosed;

  geometry_msgs::PoseStamped currentGoal;
  ros::Publisher goalPublisher;
  ros::Timer pubTimer;

  moveit::planning_interface::MoveGroupInterface group;
  ros::ServiceClient psDiffClient;
  ros::ServiceClient getPSClient;
};
