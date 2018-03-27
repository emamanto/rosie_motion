#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>

#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "moveit_msgs/CollisionObject.h"

class ArmController {
public:
  typedef std::vector<moveit::planning_interface::MoveGroupInterface::Plan> PlanVector;

  ArmController();
  void setHumanChecks(bool on) { checkPlans = on; }

  std::string armPlanningFrame();

  void buildCollisionScene(std::vector<moveit_msgs::CollisionObject> cos);
  void clearCollisionScene();

  void closeGripper();
  void openGripper();

private:
  void setGripperTo(float m);

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
