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

bool ArmController::pickUp(tf2::Transform objXform,
                           std::vector<std::pair<tf2::Transform,
                           tf2::Transform> > graspList) {
  tf2::Transform firstPose = objXform*graspList.at(0).first;
  if (!planToXform(firstPose)) return false;
  if (!executeCurrentPlan()) return false;

  openGripper();

  if (!planStraightLineMotion(objXform*graspList.at(0).second)) return false;
  if (!executeCurrentPlan()) return false;

  closeGripper();

  // if (gripperClosed) {
  //   ROS_INFO("Robot seems to have missed block %s", id.c_str());
  //   std::stringstream ss;
  //   ss << id;
  //   std::vector<std::string> missed;
  //   missed.push_back(ss.str());
  //   //scene.removeCollisionObjects(missed);
  //   ros::Duration(0.5).sleep();

  //   ROS_INFO("Arm will return home because grabbing failed");
  //   failureReason = "grasping";
  //   state = FAILURE;
  // }

  reverseCurrentPlan();
  if (!executeCurrentPlan()) return false;

  if (!homeArm()) return false;

  return true;
}

bool ArmController::putDownHeldObj(std::vector<tf2::Transform> targets) {
  return true;
}

bool ArmController::homeArm() {
  std::vector<double> joints = std::vector<double>();
  joints.push_back(1.32);
  joints.push_back(0.7);
  joints.push_back(0.0);
  joints.push_back(-2.0);
  joints.push_back(0.0);
  joints.push_back(-0.57);
  joints.push_back(0.0);
  group.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan homePlan;
  group.setStartStateToCurrentState();
  if (!group.plan(homePlan)) return false;

  if (!safetyCheck()) return false;
  currentPlan = homePlan;

  if (!executeCurrentPlan()) return false;
  return true;
}

bool ArmController::planToXform(tf2::Transform t) {
  group.setStartStateToCurrentState();
  geometry_msgs::Pose target;
  target.orientation = tf2::toMsg(t.getRotation());
  target.position.x = t.getOrigin().x();
  target.position.y = t.getOrigin().y();
  target.position.z = t.getOrigin().z();
  group.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan mp;
  moveit::planning_interface::MoveItErrorCode success = group.plan(mp);

  // To stop it thinking it's successful if postprocessing is the problem
  if (mp.trajectory_.joint_trajectory.points.empty() &&
      mp.trajectory_.multi_dof_joint_trajectory.points.empty()) {
    success = false;
  }

  currentPlan = mp;
  return (bool)success;
}

double ArmController::planStraightLineMotion(tf2::Transform target) {
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(group.getCurrentPose().pose);

  geometry_msgs::Pose p2;
  p2.orientation = tf2::toMsg(target.getRotation());
  p2.position.x = target.getOrigin().x();
  p2.position.y = target.getOrigin().y();
  p2.position.z = target.getOrigin().z();
  waypoints.push_back(p2);

  group.setStartStateToCurrentState();
  moveit_msgs::RobotTrajectory lineTraj;
  double frac = group.computeCartesianPath(waypoints,
                                           0.01, 0.0,
                                           lineTraj,
                                           false);

  currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
  currentPlan.trajectory_ = lineTraj;

  return frac;
}

bool ArmController::executeCurrentPlan() {
  if (!safetyCheck()) {
    return false;
  }

  moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(currentPlan);
  if (!moveSuccess) {
    ROS_INFO("Execution failed with error code %d", moveSuccess.val);
    return false;
  }
  return true;
}

bool ArmController::safetyCheck() {
  if (!checkPlans) return true;
  std::string input;
  std::cout << "Is this motion plan okay? ";
  std::cin >> input;

  if (input.find("y")!=std::string::npos)
    {
      ROS_INFO("Plan accepted; starting execution");
      return true;
    }

  if (input.find("n")!=std::string::npos)
    {
      ROS_INFO("Plan rejected; cancelling execution");
      return false;
    }

  ROS_INFO("Confusing input to safety check; cancelling execution");
  return false;
}

void ArmController::reverseCurrentPlan() {
  moveit_msgs::RobotTrajectory reversed;
  moveit_msgs::RobotTrajectory curTraj = currentPlan.trajectory_;

  std::vector<trajectory_msgs::JointTrajectoryPoint> rps;
  for (std::vector<trajectory_msgs::JointTrajectoryPoint>::reverse_iterator i =
         curTraj.joint_trajectory.points.rbegin();
       i!= curTraj.joint_trajectory.points.rend(); i++) {
    rps.push_back(*i);
  }
  reversed.joint_trajectory.points = rps;

  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> mdrps;
  for (std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::reverse_iterator i =
         curTraj.multi_dof_joint_trajectory.points.rbegin();
       i!= curTraj.multi_dof_joint_trajectory.points.rend(); i++) {
    mdrps.push_back(*i);
  }
  reversed.multi_dof_joint_trajectory.points = mdrps;

  reversed.joint_trajectory.header = curTraj.joint_trajectory.header;
  reversed.multi_dof_joint_trajectory.header = curTraj.multi_dof_joint_trajectory.header;

  reversed.joint_trajectory.joint_names = curTraj.joint_trajectory.joint_names;
  reversed.multi_dof_joint_trajectory.joint_names = curTraj.multi_dof_joint_trajectory.joint_names;

  currentPlan.trajectory_ = reversed;
}
