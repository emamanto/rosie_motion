#include "ArmController.h"

ArmController::ArmController(bool humanChecksPlans) : state(WAIT),
                                                      armHomeState(true),
                                                      failureReason("none"),
                                                      numRetries(3),
                                                      grabbedObject("none"),
                                                      checkPlans(humanChecksPlans),
                                                      gripperClosed(false),
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


