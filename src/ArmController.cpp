#include "ArmController.h"

ArmController::ArmController() : numRetries(3),
                                 grabbedObject("none"),
                                 checkPlans(true),
                                 fingerToWrist(-0.16645, 0, 0),
                                 group("arm"),
                                 gripper("gripper_controller/gripper_action", true)
{
  group.setMaxVelocityScalingFactor(0.4);
  gripper.waitForServer();
  closeGripper();

}

std::string ArmController::armPlanningFrame() {
  return group.getPlanningFrame();
}

void ArmController::buildCollisionScene(std::vector<moveit_msgs::CollisionObject> cos) {
  clearCollisionScene();
  scene.addCollisionObjects(cos);
}

void ArmController::clearCollisionScene() {
  std::vector<std::string> known = scene.getKnownObjectNames();
  scene.removeCollisionObjects(known);
  ros::Duration(0.1).sleep();
}

void ArmController::closeGripper() {
  setGripperTo(0.0);
}

void ArmController::openGripper() {
  setGripperTo(0.1);
}

void ArmController::setGripperTo(float m) {
  control_msgs::GripperCommandGoal gripperGoal;
  gripperGoal.command.max_effort = 0.0;
  gripperGoal.command.position = m;

  gripper.sendGoal(gripperGoal);
  gripper.waitForResult(ros::Duration(2.0));
}


