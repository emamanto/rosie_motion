#include "ArmController.h"

double ArmController::jointLength(moveit_msgs::RobotTrajectory traj) {
    if (traj.multi_dof_joint_trajectory.points.size() > 0) {
        ROS_WARN("This is a multi DOF trajectory!");
        return 0;
    }

    if (traj.joint_trajectory.points.size() == 0) return 0;

    double jl = 0;
    for (int i = 1; i < traj.joint_trajectory.points.size(); i++) {
        for (int j = 0; j < traj.joint_trajectory.points[i].positions.size(); j++) {
            jl +=  fabs(traj.joint_trajectory.points[i].positions[j] -
                        traj.joint_trajectory.points[i-1].positions[j]);
        }
    }
    return jl;
}

double ArmController::execTime(moveit_msgs::RobotTrajectory traj) {
    if (traj.multi_dof_joint_trajectory.points.size() > 0) {
        ROS_WARN("This is a multi DOF trajectory!");
        return 0;
    }

    if (traj.joint_trajectory.points.size() == 0) return 0;

    int lastPose = traj.joint_trajectory.points.size() - 1;
    return traj.joint_trajectory.points[lastPose].time_from_start.toSec();
}

double ArmController::handLength(moveit_msgs::RobotTrajectory traj) {
    if (traj.multi_dof_joint_trajectory.points.size() > 0) {
        ROS_WARN("This is a multi DOF trajectory!");
        return 0;
    }

    if (traj.joint_trajectory.points.size() == 0) return 0;

    robot_model::RobotModelConstPtr rm = group.getRobotModel();
    double approxHandDist = 0;
    Eigen::Vector3d prev;

    for (int i = 0; i < traj.joint_trajectory.points.size(); i++) {
        moveit::core::RobotState rs2(rm);
        rs2.setVariablePositions(traj.joint_trajectory.joint_names,
                                 traj.joint_trajectory.points[i].positions);
        rs2.update();
        Eigen::Affine3d xform2 = rs2.getGlobalLinkTransform(group.getEndEffectorLink());
        Eigen::Vector3d cur(xform2.translation());

        if (i > 0) {
            double dist = sqrt(pow(cur[0] - prev[0], 2) +
                               pow(cur[1] - prev[1], 2) +
                               pow(cur[2] - prev[2], 2));
            approxHandDist += dist;
        }

        prev = cur;
    }

    return approxHandDist;
}

std::vector<double> ArmController::clearanceData(moveit_msgs::RobotTrajectory traj) {
    if (traj.multi_dof_joint_trajectory.points.size() > 0) {
        ROS_WARN("This is a multi DOF trajectory!");
        return std::vector<double>();
    }

    std::vector<double> dataVals;
    if (traj.joint_trajectory.points.size() == 0) {
        dataVals.push_back(0.0);
        dataVals.push_back(0.0);
        return dataVals;
    }

    // Only care about clearance if we get within 5cm of something
    double MAX_DIST_FOR_AVG = 0.05;

    psm->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO ps(psm);
    robot_state::RobotState rs(*group.getCurrentState(0.1));
    collision_detection::CollisionRobotConstPtr cr = ps->getCollisionRobotUnpadded();

    collision_detection::AllowedCollisionMatrix acm;
    acm.clear();
    acm.setDefaultEntry("ground", true);

    double distSum = 0;
    double distMin = 1000;
    for (int i = 1; i < traj.joint_trajectory.points.size() - 1; i++) {
        rs.setVariablePositions(traj.joint_trajectory.joint_names,
                                traj.joint_trajectory.points[i].positions);
        rs.updateCollisionBodyTransforms();

        collision_detection::CollisionRequest req;
        req.group_name = "arm";
        req.distance = true;
        req.verbose = true;
        collision_detection::CollisionResult res;
        ps->getCollisionWorld()->checkRobotCollision(req, res,
                                                     *cr, rs, acm);
        double resultDist = (res.distance < 0 ? 0 : res.distance);

        if (resultDist < MAX_DIST_FOR_AVG) {
            distSum += (MAX_DIST_FOR_AVG - resultDist)/MAX_DIST_FOR_AVG;
        }
        if (resultDist < distMin) distMin = resultDist;
    }

    double finalAvg = distSum / traj.joint_trajectory.points.size();
    dataVals.push_back(distMin);
    dataVals.push_back(finalAvg);
    return dataVals;
}

ArmController::ArmController(ros::NodeHandle& nh) : numRetries(2),
                                                    checkPlans(true),
                                                    stomp(false),
                                                    group("arm"),
                                                    gripper("gripper_controller/gripper_action", true)
{
    std::time_t t;
    std::time(&t);
    struct std::tm* lt = std::localtime(&t);
    std::stringstream ss;
    ss << "logfile" << lt->tm_mon + 1 << lt->tm_mday << lt->tm_hour
       << lt->tm_min << lt->tm_sec << ".txt";
    logFileName = ss.str();
    std::ofstream ofs;
    ofs.open(logFileName);
    ofs << "TIME " << lt->tm_mon + 1 << "/" << lt->tm_mday << ", "
        << lt->tm_hour << ":" << lt->tm_min << ":" << lt->tm_sec << std::endl;
    ofs.close();
    ROS_INFO("Logging motion history to %s", logFileName.c_str());

    group.setMaxVelocityScalingFactor(0.4);
    group.setEndEffectorLink("gripper_link");
    gripper.waitForServer();
    closeGripper();
    grabbedObject.id = "NONE";

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

    planRequestClient = nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
    planRequestClient.waitForExistence();

    psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
}

void ArmController::setPlannerName(std::string n) {
    plannerName = n;
    if (plannerName == "stomp") {
        if (!stomp) {
            ROS_INFO("Attempting to set STOMP as planner when using OMPL!");
            plannerName = "rrtc";
            group.setPlannerId("RRTConnectkConfigDefault");
        }
    } else if (plannerName.find("rrt") != std::string::npos && stomp) {
      ROS_INFO("Attempting to set an RRT-based alg as planner when using STOMP!");
      plannerName = "stomp";
    } else if (plannerName == "rrtc") {
      group.setPlannerId("RRTConnectkConfigDefault");
    } else if (plannerName == "rrtstar") {
      group.setPlannerId("RRTstarkConfigDefault");
    } else if (plannerName == "rrtstarclear") {
      group.setPlannerId("RRTstarkConfigClearance");
    } else if (plannerName == "infrrtstar") {
      group.setPlannerId("InfRRTstarkConfigDefault");
    } else if (plannerName == "infrrtstarclear") {
      group.setPlannerId("InfRRTstarkConfigClearance");
    } else if (plannerName == "trrt") {
      group.setPlannerId("TRRTkConfigDefault");
    } else if (plannerName == "trrtclear") {
      group.setPlannerId("TRRTkConfigClearance");
    }

    ROS_INFO("ArmController set up to use %s planner", plannerName.c_str());
}

void ArmController::setPlanningTime(double t) {
  planningTime = t;
  group.setPlanningTime(planningTime);
  ROS_INFO("ArmController will plan for %f s", planningTime);
}

std::string ArmController::armPlanningFrame() {
    return group.getPlanningFrame();
}

void ArmController::setGripperClosed(bool isClosed) {
    if (!gripperClosed && isClosed) {
        ROS_INFO("Gripper now CLOSED");
    }
    if (gripperClosed && !isClosed) {
        ROS_INFO("Gripper now OPEN");
    }
    gripperClosed = isClosed;
}

void ArmController::updateCollisionScene(std::vector<moveit_msgs::CollisionObject> cos) {
    moveit_msgs::GetPlanningScene::Request getRequest;
    moveit_msgs::GetPlanningScene::Response getResponse;

    getRequest.components.components = getRequest.components.WORLD_OBJECT_GEOMETRY;
    if (!getPSClient.call(getRequest, getResponse)) {
        ROS_WARN("Requesting the current collision scene failed!!");
        return;
    }

    moveit_msgs::ApplyPlanningScene::Request applyRequest;
    moveit_msgs::ApplyPlanningScene::Response applyResponse;
    applyRequest.scene.is_diff = true;

    // Check if scene objects need to be updated or removed
    for (std::size_t i = 0; i < getResponse.scene.world.collision_objects.size(); i++) {
        // Don't change anything about the held object
        if (grabbedObject.id.compare(getResponse.scene.world.collision_objects[i].id) == 0) {
            continue;
        }

        // See if object in the world matches object in requested list
        bool matched = false;
        for (std::vector<moveit_msgs::CollisionObject>::iterator j = cos.begin();
             j != cos.end(); j++) {
            if (j->id.compare(getResponse.scene.world.collision_objects[i].id) != 0)
                continue;

            // If so, update its position
            matched = true;

            // Check to see if we also need to change the size, ie has its database
            // entry been edited
            bool sizeChanged = false;
            for (int p = 0; p < j->primitives.size(); p++) {
                for (int k = 0; k < j->primitives[p].dimensions.size(); k++) {
                    if (j->primitives[p].dimensions[k] !=
                        getResponse.scene.world.collision_objects[i].primitives[p].dimensions[k]) {
                        sizeChanged = true;
                        break;
                    }
                }
            }

            // To change the size, you have to remove it and add again
            if (sizeChanged) {
                moveit_msgs::CollisionObject removal;
                removal.header.frame_id = armPlanningFrame();
                removal.id = getResponse.scene.world.collision_objects[i].id;
                removal.operation = removal.REMOVE;
                applyRequest.scene.world.collision_objects.push_back(removal);
                moveit_msgs::CollisionObject newObj = *j;
                newObj.header.frame_id = armPlanningFrame();
                newObj.operation = newObj.ADD;
                applyRequest.scene.world.collision_objects.push_back(newObj);
            }
            // Otherwise you can just move it
            else {
                moveit_msgs::CollisionObject edits;
                edits.header.frame_id = armPlanningFrame();
                edits.id = j->id;
                edits.operation = edits.MOVE;
                for (int p = 0; p < j->primitive_poses.size(); p++) {
                    edits.primitive_poses.push_back(j->primitive_poses[p]);
                }
                applyRequest.scene.world.collision_objects.push_back(edits);
            }
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
        if (grabbedObject.id.compare(j->id) == 0) {
            continue;
        }

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
        ROS_WARN("Updating the collision scene failed!!");
    }
}

void ArmController::attachToGripper(std::string objName) {
    moveit_msgs::GetPlanningScene::Request getRequest;
    moveit_msgs::GetPlanningScene::Response getResponse;

    getRequest.components.components = getRequest.components.WORLD_OBJECT_GEOMETRY;
    if (!getPSClient.call(getRequest, getResponse)) {
        ROS_WARN("Requesting the current collision scene failed!!");
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
        ROS_WARN("Updating the collision scene with attached object failed!!");
    }
    else {
        grabbedObject = co;
    }
}

void ArmController::detachHeldObject() {
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

    psDiffClient.call(applyRequest, applyResponse);
    if (!applyResponse.success) {
        ROS_WARN("Updating the collision scene with attached object failed!!");
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
    tf2::Transform firstPose;
    int graspIndex = -1;
    for (int i = 0; i < graspList.size(); i++) {
        firstPose = objXform*graspList.at(i).first;
        if (planToXform(firstPose, numRetries)) {
            ROS_INFO("Grasp %i succeeded!!", i);
            graspIndex = i;
            break;
        }
    }

    if (graspIndex == -1) return false;
    if (!executeCurrentPlan()) return false;

    ros::Duration(1.0).sleep();
    openGripper();
    ros::Duration(0.5).sleep();

    //setCurrentGoalTo(objXform*graspList.at(0).second);
    //if (!planStraightLineMotion(objXform*graspList.at(graspIndex).second)) return false;
    //if (!executeCurrentPlan()) return false;

    ros::Duration(0.5).sleep();
    closeGripper();
    ros::Duration(0.5).sleep();

    if (gripperClosed) {
        ROS_INFO("Arm will return home because grasping the object failed.");
        if (!homeArm()) {
            ROS_INFO("Arm failed to return home.");
        }
        return false;
    }

    attachToGripper(objName);
    usedGrasp = graspList.at(graspIndex);
    prevObjRotation = tf2::Transform(objXform.getRotation());

    //setCurrentGoalTo(firstPose);
    //if (!planStraightLineMotion(firstPose)) return false;
    //if (!executeCurrentPlan()) return false;

    if (!homeArm()) return false;

    if (gripperClosed) {
        ROS_INFO("Arm seems to have dropped the object it was holding.");
        detachHeldObject();
        grabbedObject = moveit_msgs::CollisionObject();
        grabbedObject.id = "NONE";
        return false;
    }

    return true;
}

bool ArmController::putDownHeldObj(std::vector<tf2::Transform> targets) {
    if (grabbedObject.id == "NONE") {
        ROS_WARN("Trying to put down an object with no object in hand!!");
        return false;
    }

    // NEEDS FIXING
    // Targets refer to tabletop height, we want object height, and also
    // you want to always drop off at a little higher than you picked up...
    float zAdjust = 0.5;
    shape_msgs::SolidPrimitive sp = grabbedObject.primitives[0];
    if (grabbedObject.primitives[0].type == grabbedObject.primitives[0].BOX) {
        zAdjust *= (sp.dimensions[2] + 0.05);
    }
    else if (grabbedObject.primitives[0].type == grabbedObject.primitives[0].CYLINDER) {
        zAdjust *= (sp.dimensions[0] + 0.05);
    }

    for(std::vector<tf2::Transform>::iterator i = targets.begin();
        i != targets.end(); i++) {
        tf2::Vector3 v = i->getOrigin();
        v.setZ(v.z() + zAdjust);
        i->setOrigin(v);
    }

    bool ok = false;
    tf2::Transform firstPose;
    int targetIndex = 0;
    for (std::vector<tf2::Transform>::iterator i = targets.begin();
         i != targets.end(); i++) {
        firstPose = (*i)*prevObjRotation*usedGrasp.first;
        setCurrentGoalTo(firstPose);
        if (planToXform(firstPose, numRetries)) {
            ok = true;
            break;
        }
        targetIndex++;
    }

    if (!ok) return false;

    if (!executeCurrentPlan()) return false;
    ros::Duration(1.0).sleep();

    tf2::Transform secondPose = targets.at(targetIndex)*prevObjRotation*usedGrasp.second;
    setCurrentGoalTo(secondPose);
    if (!planStraightLineMotion(secondPose)) return false;
    if (!executeCurrentPlan()) return false;

    ros::Duration(0.5).sleep();
    openGripper();
    ros::Duration(0.5).sleep();

    detachHeldObject();
    setCurrentGoalTo(firstPose);
    if (!planStraightLineMotion(firstPose)) return false;
    if (!executeCurrentPlan()) return false;

    ros::Duration(0.5).sleep();
    closeGripper();
    ros::Duration(0.5).sleep();

    if (!homeArm()) return false;

    return true;
}

bool ArmController::pointTo(tf2::Transform objXform, float objHeight) {
    tf2::Quaternion downRot;
    downRot.setRPY(0, M_PI/2, 0);
    tf2::Transform pointXform = tf2::Transform(downRot,
                                               tf2::Vector3(0.0,
                                                            0.0,
                                                            objHeight + 0.18));

    tf2::Transform firstPose = objXform*pointXform;
    setCurrentGoalTo(firstPose);

    if (!planToXform(firstPose, numRetries)) return false;
    if (!executeCurrentPlan()) return false;

    ros::Duration(1.0).sleep();

    if (!homeArm()) return false;

    return true;
}

bool ArmController::planToTargetList(std::vector<tf2::Transform> targets,
                                     int numTrials) {
    std::ofstream ofs;
    ofs.open(logFileName, std::ofstream::out | std::ofstream::app);
    ofs << std::endl << "BEGIN LIST " << std::endl;
    ofs.close();

    bool ok = false;
    for (std::vector<tf2::Transform>::iterator i = targets.begin();
         i != targets.end(); i++) {
        //setCurrentGoalTo(*i);
        for (int j = 0; j < numTrials; j++) {
            if (planToXform(*i, 1)) {
                ok = true;
            }
        }
    }
    return ok;
}

// This is no longer really checking IK, but oh well
bool ArmController::checkIKPose(tf2::Transform blockXform) {
    geometry_msgs::Pose bp;
    bp.orientation = tf2::toMsg(blockXform.getRotation());
    bp.position.x = blockXform.getOrigin().x();
    bp.position.y = blockXform.getOrigin().y();
    bp.position.z = blockXform.getOrigin().z();

    if (!group.setJointValueTarget(bp)) {
      ROS_INFO("No IK solution!");
      return false;
    }
    else {
        robot_state::RobotState goalCopy(group.getJointValueTarget());
        goalCopy.setVariablePosition("torso_lift_joint", 0.2);
        goalCopy.update();

        psm->requestPlanningSceneState();
        planning_scene_monitor::LockedPlanningSceneRO ps(psm);

        collision_detection::CollisionRequest req;
        req.group_name = "arm";
        req.verbose = true;
        collision_detection::CollisionResult res;
        collision_detection::AllowedCollisionMatrix acm;
        acm.clear();
        acm.setDefaultEntry("ground", true);
        acm.setEntry(goalCopy.getRobotModel()->getLinkModelNames(),
                     goalCopy.getRobotModel()->getLinkModelNames(), true);

        ps->checkCollision(req, res, goalCopy, acm);

        if (res.collision) {
            ROS_INFO("IK solution in collision!");
            return false;
        }
    }

    if (!planToXform(blockXform, 1)) {
        ROS_INFO("No solution found.");
        return false;
    }
    else {
        ROS_INFO("Successful planning!");
        return true;
    }
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
    bool ok = true;

    if (stomp) {
        group.setStartState(*group.getCurrentState());
    } else {
        group.setStartStateToCurrentState();
    }
    if (!group.plan(homePlan)) ok = false;

    // To stop it thinking it's successful if null plan
    if (jointLength(homePlan.trajectory_) < 0.0001 ) {
        homePlan = moveit::planning_interface::MoveGroupInterface::Plan();
        ok = false;
    }
    writeHomeQuery(homePlan);
    if (!ok) return false;

    if (!safetyCheck()) return false;
    currentPlan = homePlan;
    if (!executeCurrentPlan()) return false;
    return true;
}

bool ArmController::planToXform(tf2::Transform t, int n) {
    if (stomp) {
        group.setStartState(*group.getCurrentState());
    } else {
        group.setStartStateToCurrentState();
    }

    geometry_msgs::Pose target;
    target.orientation = tf2::toMsg(t.getRotation());
    target.position.x = t.getOrigin().x();
    target.position.y = t.getOrigin().y();
    target.position.z = t.getOrigin().z();

    if (stomp) {
      group.setJointValueTarget(target);
    } else {
      group.setPoseTarget(target);
    }
    ros::Duration(0.1).sleep();

    int numTries = 0;
    while (numTries < n && !planToXformInner(t)) {
        numTries++;
    }

    if (numTries == n) return false;

    return true;
}

bool ArmController::planToXformInner(tf2::Transform t) {
  setCurrentGoalTo(t);

  moveit::planning_interface::MoveGroupInterface::Plan mp;
  moveit::planning_interface::MoveItErrorCode ok = group.plan(mp);

  // To stop it thinking it's successful if null plan
  if (jointLength(mp.trajectory_) < 0.0001 ) {
    mp = moveit::planning_interface::MoveGroupInterface::Plan();
    ok = false;
  }

  writeQuery(t, mp);

  currentPlan = mp;
  return (bool)ok;
}

bool ArmController::planToRegionAsList(float xD, float yD, float zD,
                                       geometry_msgs::Pose p, int numTrials) {
    std::ofstream ofs;
    ofs.open(logFileName, std::ofstream::out | std::ofstream::app);
    ofs << std::endl << "BEGIN LIST REG" << std::endl;
    ofs.close();

    for (int i = 0; i < numTrials; i++) {
        planToRegion(xD, yD, zD, p);
    }

    return true;
}

bool ArmController::planToRegion(float xD, float yD, float zD, geometry_msgs::Pose p) {
    moveit_msgs::GetMotionPlan::Request planRequest;
    moveit_msgs::GetMotionPlan::Response planResponse;
    if (stomp) {
        group.setStartState(*group.getCurrentState());
        group.setJointValueTarget(p);
    } else {
        group.setStartStateToCurrentState();
    }

    planRequest.motion_plan_request.group_name = group.getName();
    planRequest.motion_plan_request.num_planning_attempts = 1;
    planRequest.motion_plan_request.allowed_planning_time = planningTime;

    if (plannerName == "rrtc") {
        planRequest.motion_plan_request.planner_id = "RRTConnectkConfigDefault";
    } else if (plannerName == "rrtstar") {
        planRequest.motion_plan_request.planner_id = "RRTstarkConfigDefault";
    } else if (plannerName == "rrtstarclear") {
        planRequest.motion_plan_request.planner_id = "RRTstarkConfigClearance";
    } else if (plannerName == "trrt") {
        planRequest.motion_plan_request.planner_id = "TRRTkConfigDefault";
    } else if (plannerName == "trrtclear") {
        planRequest.motion_plan_request.planner_id = "TRRTkConfigClearance";
    }
    planRequest.motion_plan_request.goal_constraints.clear();

    moveit_msgs::Constraints cm;
    cm.name = "goalregion";
    cm.joint_constraints.clear();
    cm.position_constraints.clear();
    cm.orientation_constraints.clear();
    cm.visibility_constraints.clear();

    moveit_msgs::PositionConstraint pc;
    pc.header.frame_id = group.getPlanningFrame();
    pc.link_name = group.getEndEffectorLink();
    pc.target_point_offset.x = 0;
    pc.target_point_offset.y = 0;
    pc.target_point_offset.z = 0;
    pc.constraint_region.primitives.clear();
    pc.constraint_region.primitive_poses.clear();
    pc.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.push_back(xD);
    box.dimensions.push_back(yD);
    box.dimensions.push_back(zD);

    pc.constraint_region.primitives.push_back(box);
    pc.constraint_region.primitive_poses.push_back(p);
    cm.position_constraints.push_back(pc);

    if (stomp) {
      robot_state::RobotState ss(*group.getCurrentState());
      moveit::core::robotStateToRobotStateMsg(ss,
                                              planRequest.motion_plan_request.start_state,
                                              true);
      ros::Duration(0.1).sleep();

      moveit_msgs::OrientationConstraint oc;
      oc.header.frame_id = group.getPlanningFrame();
      oc.link_name = group.getEndEffectorLink();

      geometry_msgs::Quaternion q = tf2::toMsg(tf2::Quaternion::getIdentity());
      oc.orientation = q;
      oc.absolute_x_axis_tolerance = 2*M_PI;
      oc.absolute_y_axis_tolerance = 2*M_PI;
      oc.absolute_z_axis_tolerance = 2*M_PI;
      oc.weight = 0;
      cm.orientation_constraints.push_back(oc);

      std::vector<std::string> jns = group.getJointValueTarget().getVariableNames();
      for(std::vector<std::string>::iterator i = jns.begin(); i != jns.end(); i++) {
        moveit_msgs::JointConstraint jc;
        jc.joint_name = *i;
        jc.position = group.getJointValueTarget().getVariablePosition(*i);
        jc.tolerance_above = 0;
        jc.tolerance_below = 0;
        jc.weight = 0;
        cm.joint_constraints.push_back(jc);
        ros::Duration(0.1).sleep();
      }
    }

    planRequest.motion_plan_request.goal_constraints.push_back(cm);
    ros::Duration(0.5).sleep();

    if (!planRequestClient.call(planRequest, planResponse)) {
        ROS_WARN("Requesting the current collision scene failed!!");
        return false;
    }

    currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
    tf2::Transform actualGoal;

    // success
    if (planResponse.motion_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        currentPlan.trajectory_ = planResponse.motion_plan_response.trajectory;
        currentPlan.start_state_ = planResponse.motion_plan_response.trajectory_start;
        currentPlan.planning_time_ = planResponse.motion_plan_response.planning_time;

        robot_trajectory::RobotTrajectory rt(group.getRobotModel(), group.getName());
        robot_state::RobotState curStateCopy(*group.getCurrentState());
        curStateCopy.update();
        rt.setRobotTrajectoryMsg(curStateCopy, currentPlan.trajectory_);

        Eigen::Affine3d endEE = rt.getLastWayPoint().getGlobalLinkTransform("gripper_link");
        Eigen::Vector3d eeXYZ = endEE.translation();
        Eigen::Quaterniond eeQ(endEE.rotation());

        actualGoal.setOrigin(tf2::Vector3(eeXYZ(0), eeXYZ(1), eeXYZ(2)));
        actualGoal.setRotation(tf2::Quaternion(eeQ.x(), eeQ.y(), eeQ.z(), eeQ.w()));
    } else {
        actualGoal.setOrigin(tf2::Vector3(xD, yD, zD));
        actualGoal.setRotation(tf2::Quaternion::getIdentity());
    }

    writeQuery(actualGoal, currentPlan);
    //setCurrentGoalTo(actualGoal);
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
        ROS_WARN("Execution failed with error code %d", moveSuccess.val);
        return false;
    }
    return true;
}

void ArmController::writeQuery(tf2::Transform t,
                               moveit::planning_interface::MoveGroupInterface::Plan p) {
    std::ofstream ofs;
    ofs.open(logFileName, std::ofstream::out | std::ofstream::app);

    ofs << "AL " << plannerName;
    ofs << " TO "
        << t.getOrigin().x() << " "
        << t.getOrigin().y() << " "
        << t.getOrigin().z();
    ofs << " TR "
        << t.getRotation().x() << " "
        << t.getRotation().y() << " "
        << t.getRotation().z() << " "
        << t.getRotation().w();
    ofs << " TI ";
    if (jointLength(p.trajectory_) > 0) {
         ofs << p.planning_time_;
    } else {
        ofs << "-1";
    }
    writeTrajectoryInfo(ofs, p.trajectory_);
    ofs << std::endl;

    ofs.close();
}

void ArmController::writeHomeQuery(moveit::planning_interface::MoveGroupInterface::Plan p) {
    std::ofstream ofs;
    ofs.open(logFileName, std::ofstream::out | std::ofstream::app);

    ofs << "HOME POS ";
    ofs << "AL " << plannerName;
    ofs << " TI ";
    if (jointLength(p.trajectory_) > 0) {
         ofs << p.planning_time_;
    } else {
        ofs << "-1";
    }
    writeTrajectoryInfo(ofs, p.trajectory_);
    ofs << std::endl;

    ofs.close();
}

void ArmController::writeTrajectoryInfo(std::ofstream& ofs,
                                        moveit_msgs::RobotTrajectory& traj) {
    ofs << " SUCC ";
    if (jointLength(traj) > 0) {
        ofs << "Y";
    } else {
        ofs << "N";
    }
    ofs << " JL " << jointLength(traj);
    ofs << " ET " << execTime(traj);
    ofs << " HL " << handLength(traj);
    std::vector<double> cd = clearanceData(traj);
    ofs << " MC " << cd[0];
    ofs << " CA " << cd[1];
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
