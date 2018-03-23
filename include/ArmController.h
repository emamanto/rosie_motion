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
  enum ActionState {WAIT,
                    HOME,
                    GRAB,
                    POINT,
                    DROP,
                    PUSH,
                    FAILURE,
                    SCENE};

  static std::string asToString(ActionState a)
  {
    switch (a)
      {
      case WAIT: return "WAIT";
      case HOME: return "HOME";
      case GRAB: return "GRAB";
      case POINT: return "POINT";
      case DROP: return "DROP";
      case PUSH: return "PUSH";
      case FAILURE: return "FAILURE";
      case SCENE: return "SCENE";
      default: return "WTF";
      }
  }

  ArmController(bool humanChecksPlans);

  std::string armPlanningFrame();

  void closeGripper();
  void openGripper();

private:
  void setGripperTo(float m);

  ActionState state;
  bool armHomeState;

  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  std::string failureReason;
  int numRetries;
  std::string grabbedObject;
  std::vector<float> grabbedObjSize;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
  bool checkPlans;
  tf2::Vector3 fingerToWrist;

  moveit::planning_interface::MoveGroupInterface group;
  moveit::planning_interface::PlanningSceneInterface scene;
}
