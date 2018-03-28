#pragma once

#include <iostream>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>

#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "moveit_msgs/CollisionObject.h"



class ArmController {
public:
  typedef std::vector<moveit::planning_interface::MoveGroupInterface::Plan> PlanVector;
  typedef std::pair<tf2::Transform, tf2::Transform> GraspPair;

  ArmController();
  void setHumanChecks(bool on) { checkPlans = on; }

  std::string armPlanningFrame();

  void buildCollisionScene(std::vector<moveit_msgs::CollisionObject> cos);
  void clearCollisionScene();

  void closeGripper();
  void openGripper();

  bool pickUp(tf2::Transform objXform,
              std::vector<GraspPair> graspList);
  bool putDownHeldObj(std::vector<tf2::Transform> targets);
  bool homeArm();

private:
  void setGripperTo(float m);
  bool planToXform(tf2::Transform t);
  double planStraightLineMotion(tf2::Transform target);
  bool executeCurrentPlan();
  bool safetyCheck();
  void reverseCurrentPlan();

  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  int numRetries;
  std::string grabbedObject;
  std::vector<float> grabbedObjSize;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
  bool checkPlans;
  tf2::Vector3 fingerToWrist;

  moveit::planning_interface::MoveGroupInterface group;
  moveit::planning_interface::PlanningSceneInterface scene;
};
