#include "ArmController.h"

ArmController::ArmController(ros::NodeHandle& nh) : numRetries(3),
                                                    checkPlans(true),
                                                    group("arm"),
                                                    gripper("gripper_controller/gripper_action", true)
{
  group.setMaxVelocityScalingFactor(0.4);
  gripper.waitForServer();
  closeGripper();

  currentGoal.pose.position.x = 0;
  currentGoal.pose.position.y = 0;
  currentGoal.pose.position.z = 0;
  currentGoal.pose.orientation.w = 1.0;
  currentGoal.header.frame_id = group.getPlanningFrame();

  goalPublisher = nh.advertise<geometry_msgs::PoseStamped>("rosie_grasp_target", 10);
  pubTimer = nh.createTimer(ros::Duration(0.1),
                            &ArmController::publishCurrentGoal, this);
  psDiffClient = nh.serviceClient<moveit_msgs::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);
  psDiffClient.waitForExistence();
  getPSClient = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  getPSClient.waitForExistence();
}

std::string ArmController::armPlanningFrame() {
  return group.getPlanningFrame();
}

void ArmController::updateCollisionScene(std::vector<moveit_msgs::CollisionObject> cos) {
  moveit_msgs::GetPlanningScene::Request getRequest;
  moveit_msgs::GetPlanningScene::Response getResponse;

  getRequest.components.components = getRequest.components.WORLD_OBJECT_GEOMETRY;
  if (!getPSClient.call(getRequest, getResponse)) {
    ROS_INFO("Requesting the current collision scene failed!!");
    return;
  }

  moveit_msgs::ApplyPlanningScene::Request applyRequest;
  moveit_msgs::ApplyPlanningScene::Response applyResponse;
  applyRequest.scene.is_diff = true;

  // Check if scene objects need to be updated or removed
  for (std::size_t i = 0; i < getResponse.scene.world.collision_objects.size(); i++) {
    // Don't change anything about the held object
    if (grabbedObject.id.compare(getResponse.scene.world.collision_objects[i].id) == 0)
      continue;

    // See if object in the world matches object in requested list
    bool matched = false;
    for (std::vector<moveit_msgs::CollisionObject>::iterator j = cos.begin();
         j != cos.end(); j++) {
      if (j->id.compare(getResponse.scene.world.collision_objects[i].id) != 0)
        continue;

      // If so, update its position
      matched = true;
      moveit_msgs::CollisionObject edits;
      edits.header.frame_id = armPlanningFrame();
      edits.id = j->id;
      edits.operation = edits.MOVE;
      edits.primitive_poses.push_back(j->primitive_poses[0]);
      applyRequest.scene.world.collision_objects.push_back(edits);
      break;
    }

    // Otherwise it must have disappeared, remove it
    if (!matched) {
      moveit_msgs::CollisionObject removal;
      removal.header.frame_id = armPlanningFrame();
      removal.id = getResponse.scene.world.collision_objects[i].id;
      removal.operation = removal.REMOVE;
      applyRequest.scene.world.collision_objects.push_back(removal);
    }
  }

  // Check if vector contains entirely new objects
  for (std::vector<moveit_msgs::CollisionObject>::iterator j = cos.begin();
       j != cos.end(); j++) {
    bool isNewObj = true;
    for (std::size_t i = 0; i < getResponse.scene.world.collision_objects.size();
         i++) {
      if (j->id.compare(getResponse.scene.world.collision_objects[i].id) == 0) {
        isNewObj = false;
        break;
      }
    }

    if (!isNewObj) continue;

    // Add object to the scene if it is new
    moveit_msgs::CollisionObject newObj = *j;
    newObj.header.frame_id = armPlanningFrame();
    newObj.operation = newObj.ADD;
    applyRequest.scene.world.collision_objects.push_back(newObj);
  }

  psDiffClient.call(applyRequest, applyResponse);
  if (!applyResponse.success) {
    ROS_INFO("Updating the collision scene failed!!");
  }
}

void ArmController::attachToGripper(std::string objName) {
  moveit_msgs::GetPlanningScene::Request getRequest;
  moveit_msgs::GetPlanningScene::Response getResponse;

  getRequest.components.components = getRequest.components.WORLD_OBJECT_GEOMETRY;
  if (!getPSClient.call(getRequest, getResponse)) {
    ROS_INFO("Requesting the current collision scene failed!!");
    return;
  }

  moveit_msgs::CollisionObject co;
  for (std::size_t i = 0; i < getResponse.scene.world.collision_objects.size(); i++) {
    if (objName.compare(getResponse.scene.world.collision_objects[i].id) == 0) {
        co = getResponse.scene.world.collision_objects[i];
        break;
    }
  }

  moveit_msgs::ApplyPlanningScene::Request applyRequest;
  moveit_msgs::ApplyPlanningScene::Response applyResponse;
  applyRequest.scene.is_diff = true;

  applyRequest.scene.world.collision_objects.clear();
  applyRequest.scene.robot_state.attached_collision_objects.clear();

  moveit_msgs::CollisionObject removal;
  removal.header.frame_id = armPlanningFrame();
  removal.id = co.id;
  removal.operation = removal.REMOVE;
  applyRequest.scene.world.collision_objects.push_back(removal);
  moveit_msgs::AttachedCollisionObject toAttach;
  toAttach.link_name = "wrist_roll_link";
  toAttach.touch_links.push_back("l_gripper_finger_link");
  toAttach.touch_links.push_back("r_gripper_finger_link");
  toAttach.touch_links.push_back("gripper_link");
  toAttach.object = co;
  toAttach.object.operation = toAttach.object.ADD;
  applyRequest.scene.robot_state.is_diff = true;
  applyRequest.scene.robot_state.attached_collision_objects.push_back(toAttach);

  psDiffClient.call(applyRequest, applyResponse);
  if (!applyResponse.success) {
    ROS_INFO("Updating the collision scene with attached object failed!!");
  }
  else {
    grabbedObject = co;
  }
}

void ArmController::detachHeldObject() {
  ROS_INFO("Detaching the object from robot");
  moveit_msgs::GetPlanningScene::Request getRequest;
  moveit_msgs::GetPlanningScene::Response getResponse;

  getRequest.components.components = getRequest.components.WORLD_OBJECT_GEOMETRY;
  if (!getPSClient.call(getRequest, getResponse)) {
    ROS_INFO("Requesting the current collision scene failed!!");
    return;
  }

  ROS_INFO("Finding which object needs to be added back...");
  moveit_msgs::CollisionObject addBack;
  for (std::size_t i = 0;
       i < getResponse.scene.robot_state.attached_collision_objects.size();
       i++) {
    if (grabbedObject.id.compare(getResponse.scene.robot_state.attached_collision_objects[i].object.id) == 0) {
      addBack = getResponse.scene.robot_state.attached_collision_objects[i].object;
      break;
    }
  }

  ROS_INFO("Setting up the scene diff... ");
  moveit_msgs::ApplyPlanningScene::Request applyRequest;
  moveit_msgs::ApplyPlanningScene::Response applyResponse;
  applyRequest.scene.is_diff = true;

  applyRequest.scene.world.collision_objects.clear();
  applyRequest.scene.robot_state.attached_collision_objects.clear();
  applyRequest.scene.robot_state.is_diff = true;

  moveit_msgs::AttachedCollisionObject toDetach;
  toDetach.object = grabbedObject;
  toDetach.object.operation = toDetach.object.REMOVE;
  applyRequest.scene.robot_state.attached_collision_objects.push_back(toDetach);

  addBack.operation = addBack.ADD;
  applyRequest.scene.world.collision_objects.push_back(addBack);

  psDiffClient.call(applyRequest, applyResponse);
  if (!applyResponse.success) {
    ROS_INFO("Updating the collision scene with attached object failed!!");
  }
  else {
    grabbedObject = moveit_msgs::CollisionObject();
    grabbedObject.id = "NONE";
  }
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
                           tf2::Transform> > graspList,
                           std::string objName) {
  tf2::Transform firstPose = objXform*graspList.at(0).first;
  setCurrentGoalTo(firstPose);

  //std::vector<std::string> known = scene.getKnownObjectNames();
  // ROS_INFO("There are %i objects in the collision scene", (int)known.size());
  // for (std::vector<std::string>::iterator it = known.begin();
  //      it != known.end(); it++) {
  //   ROS_INFO("OBJ NAME: %s", it->c_str());
  // }

  if (!planToXform(firstPose)) return false;
  if (!executeCurrentPlan()) return false;

  ros::Duration(1.0).sleep();
  openGripper();
  ros::Duration(1.0).sleep();

  setCurrentGoalTo(objXform*graspList.at(0).second);
  if (!planStraightLineMotion(objXform*graspList.at(0).second)) return false;
  if (!executeCurrentPlan()) return false;

  ros::Duration(1.0).sleep();
  closeGripper();
  ros::Duration(1.0).sleep();

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

  attachToGripper(objName);
  usedGrasp = graspList.at(0);

  setCurrentGoalTo(firstPose);
  if (!planStraightLineMotion(firstPose)) return false;
  if (!executeCurrentPlan()) return false;

  if (!homeArm()) return false;

  return true;
}

bool ArmController::putDownHeldObj(std::vector<tf2::Transform> targets) {
  if (grabbedObject.id == "NONE") {
    ROS_WARN("Trying to put down an object with no object in hand!!");
  }

  float zAdjust = 0.5;
  shape_msgs::SolidPrimitive sp = grabbedObject.primitives[0];
  if (grabbedObject.primitives[0].type == grabbedObject.primitives[0].BOX) {
    zAdjust *= (sp.dimensions[2] + 0.02);
    ROS_INFO("Adding %f to z because it's a BOX", zAdjust);
  }
  else if (grabbedObject.primitives[0].type == grabbedObject.primitives[0].CYLINDER) {
    zAdjust *= (sp.dimensions[0] + 0.02);
    ROS_INFO("Adding %f to z because it's a CYLINDER", zAdjust);
  }

  for(std::vector<tf2::Transform>::iterator i = targets.begin();
      i != targets.end(); i++) {
    tf2::Vector3 v = i->getOrigin();
    v.setZ(v.z() + zAdjust);
    i->setOrigin(v);
  }

  ROS_INFO("The z value of the target is %f", targets.at(0).getOrigin().z());
  tf2::Transform firstPose = targets.at(0)*usedGrasp.first;
  setCurrentGoalTo(firstPose);

  if (!planToXform(firstPose)) return false;
  if (!executeCurrentPlan()) return false;

  tf2::Transform secondPose = targets.at(0)*usedGrasp.second;
  setCurrentGoalTo(secondPose);
  if (!planStraightLineMotion(secondPose)) return false;
  if (!executeCurrentPlan()) return false;

  ros::Duration(1.0).sleep();
  openGripper();
  ros::Duration(1.0).sleep();

  detachHeldObject();
  setCurrentGoalTo(firstPose);
  if (!planStraightLineMotion(firstPose)) return false;
  if (!executeCurrentPlan()) return false;

  ros::Duration(1.0).sleep();
  closeGripper();
  ros::Duration(1.0).sleep();

  if (!homeArm()) return false;

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
  ros::Duration(0.1).sleep();

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

void ArmController::publishCurrentGoal(const ros::TimerEvent& e) {
  goalPublisher.publish(currentGoal);
}

void ArmController::setCurrentGoalTo(tf2::Transform t) {
  currentGoal.pose.position.x = t.getOrigin().x();
  currentGoal.pose.position.y = t.getOrigin().y();
  currentGoal.pose.position.z = t.getOrigin().z();
  currentGoal.pose.orientation = tf2::toMsg(t.getRotation());
}
