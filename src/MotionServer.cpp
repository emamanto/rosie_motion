/**
 *
 * Motion server v2.0
 *
 **/

#include <string>
#include <map>
#include <iostream>
#include <sstream>
#include <math.h>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "rosie_msgs/RobotCommand.h"
#include "rosie_msgs/RobotAction.h"
#include "gazebo_msgs/ModelStates.h"
#include "moveit_msgs/CollisionObject.h"

#include "ObjectDatabase.h"
#include "WorldObjects.h"
#include "ArmController.h"

class MotionServer
{
public:
  typedef int axis;
  static const axis X_AXIS = 0;
  static const axis Y_AXIS = 1;
  static const axis Z_AXIS = 2;

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

  MotionServer() : spinner(1, &inputQueue),
                   armSpinner(1, &armQueue),
                   tfBuf(),
                   tfListener(tfBuf),
                   lastCommandTime(0),
                   arm(n)
  {
    bool isSimRobot = false;
    // Check params and output useful info to terminal
    if (!n.getParam("/rosie_motion_server/rosie_is_sim", isSimRobot)) {
      ROS_INFO("RosieMotionServer is missing rosie_is_sim, assuming real robot.");
    }
    else if (isSimRobot == true) {
      ROS_INFO("RosieMotionServer is expecting a simulated robot.");
    }
    else {
      ROS_INFO("RosieMotionServer is expecting a real robot.");
    }

    bool checkPlans = true;
    // Also provide correct params to ArmController
    if (!n.getParam("/rosie_motion_server/human_check", checkPlans)) {
      ROS_INFO("RosieMotionServer is missing human_check parameter, keeping checks on.");
    }
    else if (checkPlans == false) {
      ROS_INFO("RosieMotionServer human checks on motion planning are OFF.");
    }
    else {
      ROS_INFO("RosieMotionServer human checks on motion planning are on.");
    }
    arm.setHumanChecks(checkPlans);

    ros::param::set("/move_group/trajectory_execution/allowed_start_tolerance", 0.0);

    ros::SubscribeOptions optionsObs =
      ros::SubscribeOptions::create<gazebo_msgs::ModelStates>("gazebo/model_states",
                                                              10,
                                                              boost::bind(&MotionServer::obsCallback,
                                                                          this, _1),
                                                              ros::VoidPtr(), &inputQueue);
    ros::SubscribeOptions optionsJoint =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states",
                                                              10,
                                                              boost::bind(&MotionServer::jointCallback,
                                                                          this, _1),
                                                              ros::VoidPtr(), &inputQueue);

    obsSubscriber = n.subscribe(optionsObs);
    jointsSubscriber = n.subscribe(optionsJoint);

    ros::SubscribeOptions optionsArm =
      ros::SubscribeOptions::create<rosie_msgs::RobotCommand>("rosie_arm_commands",
                                                              1,
                                                              boost::bind(&MotionServer::commandCallback,
                                                                          this, _1),
                                                              ros::VoidPtr(), &armQueue);
    commSubscriber = n.subscribe(optionsArm);

    statusPublisher = n.advertise<rosie_msgs::RobotAction>("rosie_arm_status", 10);
    camXPublisher = n.advertise<geometry_msgs::TransformStamped>("rosie_camera", 10);
    pubTimer = n.createTimer(ros::Duration(0.1),
                             &MotionServer::publishStatus, this);

    ROS_INFO("RosieMotionServer READY!");
  };

  void start() {
    spinner.start();
    ROS_INFO("RosieMotionServer started INPUT SPINNER");

    armSpinner.start();
    ROS_INFO("RosieMotionServer started ARM SPINNER");
  }

  void obsCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    world.update(msg);
  }

  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    int pos1 = -1;
    int pos2 = -1;
    for (int i = 0; i < msg->name.size(); i++) {
      if (msg->name[i] == "l_gripper_finger_joint") pos1 = i;
      else if (msg->name[i] == "r_gripper_finger_joint") pos2 = i;
    }
    if (pos1 == -1 || pos2 == -1) return;

    if (msg->position[pos1] < 0.005 && msg->position[pos2] < 0.005) {
      gripperClosed = true;
    } else {
      gripperClosed = false;
    }
  }

  void commandCallback(const rosie_msgs::RobotCommand::ConstPtr& msg)
  {
    if (asToString(state) == msg->action || msg->utime == lastCommandTime)
      return;

    failureReason = "none";
    lastCommandTime = msg->utime;
    if (msg->action.find("GRAB")!=std::string::npos) {
        state = GRAB;
        std::string name = msg->action.substr(msg->action.find("=")+1);
        ROS_INFO("Handling pickup command for %s", name.c_str());
        handleGrabCommand(name);
    }
    else if (msg->action.find("DROP")!=std::string::npos) {
      state = SCENE;
      arm.updateCollisionScene(getCollisionModels());

      ROS_INFO("Handling putdown command");
      state = DROP;
      std::vector<float> t = std::vector<float>();
      t.push_back(msg->dest.translation.x);
      t.push_back(msg->dest.translation.y);
      t.push_back(msg->dest.translation.z);
      handleDropCommand(t);
    }
    // else if (msg->action.find("PUSH")!=std::string::npos){
    //   state = PUSH;
    //   std::string num = msg->action.substr(msg->action.find("=")+1);
    //   ROS_INFO("Handling push command for object %s", num.c_str());

    //   // Same as grasping; need to change
    //   int idNum;
    //   std::stringstream ss(num);
    //   if (!(ss >> idNum)) {
    //     ROS_INFO("Invalid object ID number %s", num.c_str());
    //     return;
    //   }

    //   std::vector<float> t = std::vector<float>();
    //   t.push_back(msg->dest.translation.x);
    //   t.push_back(msg->dest.translation.y);
    //   t.push_back(msg->dest.translation.z);

    //   setUpScene();
    //   //handlePushCommand(num, t);
    // }
    else if (msg->action.find("POINT")!=std::string::npos) {
      state = SCENE;
      arm.updateCollisionScene(getCollisionModels());

      state = POINT;
      std::string name = msg->action.substr(msg->action.find("=")+1);
      ROS_INFO("Handling point command for object %s", name.c_str());
      handlePointCommand(name);
    }
    else if (msg->action.find("HOME")!=std::string::npos){
      ROS_INFO("Handling home command");
      state = HOME;
      arm.homeArm();
      state = WAIT;
    }
    else if (msg->action.find("RESET")!=std::string::npos){
      ROS_INFO("Handling reset command");
      state = HOME;
      arm.homeArm();
      state = WAIT;
    }
    else if (msg->action.find("SCENE")!=std::string::npos){
      ROS_INFO("Handling build scene command");
      state = SCENE;
      arm.updateCollisionScene(getCollisionModels());
      state = WAIT;
    }
    else {
      ROS_INFO("Unknown command %s received", msg->action.c_str());
      failureReason = "unknowncommand";
      state = FAILURE;
    }
  }

  // ID needs to be a substring of the object's model name
  void handleGrabCommand(std::string id)
  {
    if (!world.isInScene(id)) {
      ROS_INFO("%s is not in the scene", id.c_str());
      state = FAILURE;
      failureReason = "planning";
      return;
    }
    tf2::Transform objXform = world.worldXformTimesTrans(world.nameInScene(id));

    // Find the grasp information for this object
    std::string databaseName = "";
    if (objData.dbHasGrasps(id)) {
      databaseName = objData.findDatabaseName(world.nameInScene(id));
    }
    else {
      ROS_INFO("We do not have a grasp list for %s", id.c_str());
      state = FAILURE;
      failureReason = "planning";
      return;
    }

    arm.updateCollisionScene(getCollisionModels());
    bool success = arm.pickUp(objXform,
                              objData.getAllGrasps(databaseName),
                              world.nameInScene(id));

    if (state == GRAB) state = WAIT;
    ROS_INFO("Arm status is now WAIT");
  }

    void handleDropCommand(std::vector<float> target)
    {
      if (target[2] == -1) target[2] = world.getTableH();
      tf2::Transform targ;
      targ.setIdentity();
      targ.setOrigin(tf2::Vector3(target[0], target[1], target[2]));
      std::vector<tf2::Transform> targList;
      targList.push_back(targ);

      arm.updateCollisionScene(getCollisionModels());
      bool success = arm.putDownHeldObj(targList);

      if (state == DROP) state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    }

  // void handlePushCommand(std::string id, std::vector<float> pV)
  //   {
  //       boost::lock_guard<boost::mutex> guard(objMutex);
  //       if (objectPoses.find(id) == objectPoses.end()) {
  //         ROS_INFO("Object ID %s is not being perceived", id.c_str());
  //         failureReason = "planning";
  //         state = FAILURE;
  //         return;
  //       }

  //       if (pV[2] == -1) {
  //         ROS_INFO("No push was found; not planning motion");
  //         failureReason = "nopush";
  //         state = FAILURE;
  //         return;
  //       }

  //       if (pV[0] == 0 && pV[1] == 0) {
  //         ROS_INFO("No need to push; not planning motion");
  //         failureReason = "";
  //         state = WAIT;
  //         return;
  //       }

  //       float objYaw = tf2::getYaw(objectRotations[id]);
  //       std::vector<float> chosenPush;

  //       // Plans the reach and in/out motions at once
  //       // Prefer X push
  //       plan_vector steps;
  //       if (pV[2] == X_AXIS || (pV[2] == 2 && fabs(pV[0]) <= fabs(pV[1]))) {
  //         // Try X axis push
  //         steps = planPush(id, pV[0], X_AXIS);
  //         if (!steps.empty()) {
  //           chosenPush.push_back(pV[0]);
  //           chosenPush.push_back(0.0);
  //         }
  //         // Try Y axis push if X failed and it's available
  //         else if (steps.empty() && pV[1] != 0) {
  //           steps = planPush(id, pV[1], Y_AXIS);
  //           if (!steps.empty()) {
  //             chosenPush.push_back(0.0);
  //             chosenPush.push_back(pV[1]);
  //           }
  //         }
  //       }
  //       // Prefer Y push
  //       if (pV[2] == Y_AXIS || (pV[2] == 2 && fabs(pV[1]) < fabs(pV[0]))) {
  //         // Try Y axis push
  //         steps = planPush(id, pV[1], Y_AXIS);
  //         if (!steps.empty()) {
  //           chosenPush.push_back(0.0);
  //           chosenPush.push_back(pV[1]);
  //         }
  //         // Try X axis push if Y failed and it's available
  //         else if (steps.empty() && pV[0] != 0) {
  //           steps = planPush(id, pV[0], X_AXIS);
  //           if (!steps.empty()) {
  //             chosenPush.push_back(pV[0]);
  //             chosenPush.push_back(0.0);
  //           }
  //         }
  //       }

  //       // Total failure
  //       if (steps.empty()) {
  //         ROS_INFO("Could not find ANY plan to push, arm state is now FAILURE");
  //         failureReason = "planning";
  //         state = FAILURE;
  //         return;
  //       }

  //       // Otherwise, execute the steps we found
  //       setGripperTo(0.02);
  //       armHomeState = false;
  //       int count = 0;
  //       for (plan_vector::iterator i = steps.begin(); i != steps.end(); i++) {
  //         ros::Duration(1.0).sleep();
  //         currentPlan = *i;
  //         if (count == 2) group.setMaxVelocityScalingFactor(0.05);

  //         if (!executeCurrentPlan()) {
  //           group.setMaxVelocityScalingFactor(0.4);
  //           ROS_INFO("Arm returning home because execution failed");
  //           homeArm(true);
  //           failureReason = "execution";
  //           state = FAILURE;
  //           ROS_INFO("Arm state is now FAILURE");
  //           return;
  //         }

  //         group.setMaxVelocityScalingFactor(0.4);
  //         count++;
  //       }

  //       // Remove block from collision map
  //       std::stringstream ss;
  //       ss << id;
  //       std::vector<std::string> pushed;
  //       pushed.push_back(ss.str());
  //       scene.removeCollisionObjects(pushed);
  //       ros::Duration(0.5).sleep();

  //       // Re-add block to collision map
  //       moveit_msgs::CollisionObject pushedObj;
  //       pushedObj.header.frame_id = group.getPlanningFrame();

  //       shape_msgs::SolidPrimitive primitive;
  //       primitive.type = primitive.BOX;
  //       primitive.dimensions.resize(3);
  //       primitive.dimensions[0] = objectSizes[id][0] + 0.01;
  //       primitive.dimensions[1] = objectSizes[id][1] + 0.01;
  //       primitive.dimensions[2] = objectSizes[id][2] + 0.01;

  //       std::vector<float> trans = rotate2D(chosenPush, objYaw);
  //       geometry_msgs::Pose pushP;
  //       pushP.position.x = objectPoses[id].x() + trans[0];
  //       pushP.position.y = objectPoses[id].y() + trans[1];
  //       pushP.position.z = objectPoses[id].z();
  //       pushP.orientation.w = objectRotations[id].w();
  //       pushP.orientation.x = objectRotations[id].x();
  //       pushP.orientation.y = objectRotations[id].y();
  //       pushP.orientation.z = objectRotations[id].z();

  //       pushedObj.primitives.push_back(primitive);
  //       pushedObj.primitive_poses.push_back(pushP);
  //       pushedObj.operation = pushedObj.ADD;

  //       std::vector<moveit_msgs::CollisionObject> toAdd;
  //       toAdd.push_back(pushedObj);
  //       scene.addCollisionObjects(toAdd);
  //       ros::Duration(0.5).sleep();

  //       homeArm(true);
  //       if (state == PUSH) state = WAIT;
  //       ROS_INFO("Arm status is now WAIT");
  //   }

  // plan_vector planPush(std::string id, float dist, axis a)
  // {
  //   float x = objectPoses[id].x();
  //   float y = objectPoses[id].y();
  //   float z = objectPoses[id].z() + objectSizes[id][2]/2.0 - 0.01;
  //   float yaw = tf2::getYaw(objectRotations[id]);

  //   plan_vector plans;

  //   if (tooFar(x, y, true)) {
  //     ROS_INFO("Not planning because robot probably cannot reach (%f, %f)", x, y);
  //     plans.clear();
  //     return plans;
  //   }

  //   // Plan to the offset point
  //   std::vector<float> offsetV;
  //   if (a == Y_AXIS) offsetV.push_back(0);
  //   if (dist < 0) {
  //     offsetV.push_back(objectSizes[id][1]/2.0 + pushOffset + 0.01);
  //   }
  //   else if (dist > 0) {
  //     offsetV.push_back(-(objectSizes[id][1]/2.0 + pushOffset + 0.01));
  //   }
  //   if (a == X_AXIS) offsetV.push_back(0);

  //   std::vector<float> rotatedOffset = rotate2D(offsetV, yaw);
  //   x += rotatedOffset[0];
  //   y += rotatedOffset[1];
  //   z += 0.04;

  //   float pushYaw = yaw;
  //   if (a == Y_AXIS) pushYaw += M_PI/2.0;
  //   while (pushYaw > M_PI) pushYaw -= M_PI;

  //   // Retry the planning process several times
  //   bool success = false;
  //   int tries = 0;
  //   float handAngleDenom = 2.0;

  //   while (!success && tries < numRetries) {
  //     tries++;
  //     float handPitch = M_PI/handAngleDenom;

  //     success = planToXYZAngleTarget(x, y, z, handPitch, pushYaw);
  //     if (success) {
  //       ROS_INFO("Plan for initial reach found");
  //       plans.push_back(currentPlan);
  //     }
  //     else {
  //       ROS_INFO("Could not find a plan to reach push position at pitch %f",
  //                handPitch);
  //       plans.clear();
  //       if (dist > 0) handAngleDenom += 0.2;
  //       else handAngleDenom -= 0.2;
  //       continue;
  //     }

  //     // Setup motion to touch the side of the block
  //     std::vector<float> setupV;
  //     if (a == Y_AXIS) setupV.push_back(0);
  //     if (dist < 0) {
  //       setupV.push_back(-(pushOffset - 0.01));
  //     }
  //     else {
  //       setupV.push_back(pushOffset - 0.01);
  //     }
  //     if (a == X_AXIS) setupV.push_back(0);

  //     robot_state::RobotState setupStartState(*group.getCurrentState());
  //     setupStartState.setJointGroupPositions(group.getName(),
  //                                            currentPlan.trajectory_.joint_trajectory.points.back().positions);
  //     group.setStartState(setupStartState);

  //     std::vector<float> rotatedSetup = rotate2D(setupV, yaw);
  //     moveit_msgs::RobotTrajectory setupTraj;
  //     std::vector<geometry_msgs::Pose> setupWaypoints;
  //     setupWaypoints.push_back(xyzypTargetToPoseMsg(x, y, z, pushYaw, handPitch));

  //     geometry_msgs::Pose tp = setupWaypoints[0];
  //     tp.position.x += rotatedSetup[0];
  //     tp.position.y += rotatedSetup[1];
  //     tp.position.z -= 0.04;

  //     std::vector<float> fPos = eeFrametoFingertip(tp);
  //     if (fPos[2] < tableH) {
  //       tp.position.z += (tableH - fPos[2]);
  //     }
  //     setupWaypoints.push_back(tp);

  //     double sFrac = group.computeCartesianPath(setupWaypoints,
  //                                               0.01, 0.0,
  //                                               setupTraj,
  //                                               false);

  //     if (sFrac > 0.9) {
  //       ROS_INFO("Plan for setup motion found");
  //       currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
  //       currentPlan.trajectory_ = setupTraj;
  //       plans.push_back(currentPlan);
  //     }
  //     else {
  //       ROS_INFO("Could not find a plan for setup motion at pitch %f",
  //                handPitch);
  //       success = false;
  //       plans.clear();
  //       if (dist > 0) handAngleDenom += 0.2;
  //       else handAngleDenom -= 0.2;
  //       continue;
  //     }

  //     // Actual pushing motion
  //     std::vector<float> pushV;
  //     if (a == Y_AXIS) pushV.push_back(0);
  //     pushV.push_back(dist);
  //     if (a == X_AXIS) pushV.push_back(0);

  //     robot_state::RobotState pushStartState(*group.getCurrentState());
  //     pushStartState.setJointGroupPositions(group.getName(),
  //                                           currentPlan.trajectory_.joint_trajectory.points.back().positions);
  //     group.setStartState(pushStartState);

  //     std::vector<float> rotatedPush = rotate2D(pushV, yaw);
  //     moveit_msgs::RobotTrajectory pushTraj;
  //     std::vector<geometry_msgs::Pose> pushWaypoints;
  //     pushWaypoints.push_back(setupWaypoints.back());

  //     geometry_msgs::Pose pp = pushWaypoints[0];
  //     geometry_msgs::Pose returnto = pushWaypoints[0];

  //     pp.position.x += rotatedPush[0];
  //     pp.position.y += rotatedPush[1];
  //     pushWaypoints.push_back(pp);

  //     double pFrac = group.computeCartesianPath(pushWaypoints,
  //                                               0.01, 0.0,
  //                                               pushTraj,
  //                                               false);
  //     if (pFrac > 0.9) {
  //       ROS_INFO("Plan for push motion found");
  //       currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
  //       currentPlan.trajectory_ = pushTraj;
  //       plans.push_back(currentPlan);
  //     }
  //     else {
  //       ROS_INFO("Could not find a plan for push motion at pitch %f",
  //                handPitch);
  //       success = false;
  //       plans.clear();
  //       if (dist > 0) handAngleDenom += 0.2;
  //       else handAngleDenom -= 0.2;
  //       continue;
  //     }

  //     robot_state::RobotState outStartState(*group.getCurrentState());
  //     outStartState.setJointGroupPositions(group.getName(), currentPlan.trajectory_.joint_trajectory.points.back().positions);
  //     group.setStartState(outStartState);

  //     moveit_msgs::RobotTrajectory outTraj;
  //     std::vector<geometry_msgs::Pose> outWaypoints;
  //     outWaypoints.push_back(pushWaypoints.back());

  //     if (fabs(dist) < 0.05) {
  //       returnto.position.z += 0.04;
  //       outWaypoints.push_back(returnto);
  //     }
  //     else {
  //       geometry_msgs::Pose tp = outWaypoints[0];
  //       std::vector<float> out = rotatedPush;
  //       out[0] = 0.05 * (out[0] / dist);
  //       out[1] = 0.05 * (out[1] / dist);
  //       if (dist > 0) {
  //         tp.position.x -= out[0];
  //         tp.position.y -= out[1];
  //       } else {
  //         tp.position.x += out[0];
  //         tp.position.y += out[1];
  //       }
  //       tp.position.z += 0.04;
  //       outWaypoints.push_back(tp);
  //     }

  //     double oFrac = group.computeCartesianPath(outWaypoints,
  //                                               0.01, 0.0,
  //                                               outTraj,
  //                                               false);

  //     if (pFrac > 0.9) {
  //       ROS_INFO("Plan for outward motion found");
  //       currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
  //       currentPlan.trajectory_ = outTraj;
  //       plans.push_back(currentPlan);
  //     }
  //     else {
  //       ROS_INFO("Could not find a plan for outward motion at pitch %f",
  //                handPitch);
  //       success = false;
  //       plans.clear();
  //       if (dist > 0) handAngleDenom += 0.2;
  //       else handAngleDenom -= 0.2;
  //       continue;
  //     }
  //   }
  //   return plans;
  // }

  tf2::Quaternion yawPitchToQuat(float yaw, float pitch)
  {
    std::vector<float> res;

    tf2::Quaternion qtemp1;
    qtemp1.setRPY(0.0, 0.0, yaw);
    tf2::Quaternion qtemp2;
    qtemp2.setRPY(0.0, pitch, 0.0);

    tf2::Transform first = tf2::Transform(qtemp1);
    tf2::Transform second = tf2::Transform(qtemp2);

    tf2::Transform t = first*second;

    return t.getRotation();
  }

  void handlePointCommand(std::string id)
  {
    if (!world.isInScene(id)) {
      ROS_INFO("Object ID %s is not being perceived", id.c_str());
      return;
    }
    std::string objID = world.nameInScene(id);

    // Find the grasp information for this object
    std::string databaseName = "";
    if (objData.dbHasModel(id)) {
      databaseName = objData.findDatabaseName(objID);
    }
    else {
      ROS_INFO("We do not have a collision model for %s", id.c_str());
      state = FAILURE;
      failureReason = "planning";
      return;
    }

    float shapeHeight = 0.f;
    shape_msgs::SolidPrimitive cm = objData.getCollisionModel(databaseName);
    if (cm.type == cm.BOX) {
      shapeHeight = cm.dimensions[2];
    }
    else if (cm.type == cm.CYLINDER) {
      shapeHeight = cm.dimensions[0];
    }
    else {
      ROS_INFO("What king of object are you?!");
    }

    arm.pointTo(world.getXformOf(objID), shapeHeight);

    if (state == POINT) state = WAIT;
    ROS_INFO("Arm status is now WAIT");
  }

  void publishStatus(const ros::TimerEvent& e)
  {
    rosie_msgs::RobotAction msg = rosie_msgs::RobotAction();
    msg.utime = ros::Time::now().toNSec();
    msg.action = asToString(state).c_str();
    msg.armHome = armHomeState;
    msg.failure_reason = failureReason;
    msg.obj_id = arm.getHeld();
    statusPublisher.publish(msg);

    try {
      camXform = tfBuf.lookupTransform("base_link", "head_camera_rgb_optical_frame",
                                       ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    camXPublisher.publish(camXform);
  }

  std::vector<moveit_msgs::CollisionObject> getCollisionModels()
  {
    std::vector<moveit_msgs::CollisionObject> coList;

    std::vector<std::string> objectIDs = world.allObjectNames();
    for (std::vector<std::string>::iterator i = objectIDs.begin();
         i != objectIDs.end(); i++) {
      // Check if the fetch could actually hit this
      float dist = tf2::tf2Distance(world.getWorldXform().getOrigin(),
                                    world.getPositionOf(*i));

      if (dist > 2) {
        ROS_INFO("Object %s is out of reasonable range", i->c_str());
        continue;
      }

      tf2::Vector3 fetchCentered = world.worldXformTimesPos(*i);
      if (i->find("ground_plane") != std::string::npos) {
        moveit_msgs::CollisionObject planeobj;
        planeobj.id = "ground";

        geometry_msgs::Pose planep;
        planep.position.x = fetchCentered.x();
        planep.position.y = fetchCentered.y();
        planep.position.z = -0.025;

        planep.orientation = tf2::toMsg(world.worldXformTimesRot(*i));

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 2.0;
        primitive.dimensions[1] = 2.0;
        primitive.dimensions[2] = 0.05;

        planeobj.primitives.push_back(primitive);
        planeobj.primitive_poses.push_back(planep);
        coList.push_back(planeobj);
        continue;
      }
      else if (i->find("table") != std::string::npos) {
        moveit_msgs::CollisionObject planeobj;
        planeobj.id = "table";

        geometry_msgs::Pose planep;
        planep.position.x = fetchCentered.x();
        planep.position.y = fetchCentered.y();
        planep.position.z = world.getTableH();

        planep.orientation = tf2::toMsg(world.worldXformTimesRot(*i));

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.95;
        primitive.dimensions[1] = 0.95;
        primitive.dimensions[2] = 0.05;

        planeobj.primitives.push_back(primitive);
        planeobj.primitive_poses.push_back(planep);
        coList.push_back(planeobj);
        continue;
      }

      moveit_msgs::CollisionObject co;
      co.id = *i;

      geometry_msgs::Pose box_pose;
      box_pose.position.x = fetchCentered.x();
      box_pose.position.y = fetchCentered.y();
      box_pose.position.z = fetchCentered.z();

      geometry_msgs::Quaternion q = tf2::toMsg(world.worldXformTimesRot(*i));
      box_pose.orientation = q;

      if (objData.isInDatabase(*i)) {
        shape_msgs::SolidPrimitive primitive = objData.getCollisionModel(objData.findDatabaseName(*i));
        co.primitives.push_back(primitive);
        co.primitive_poses.push_back(box_pose);
        co.operation = co.ADD;
        coList.push_back(co);
      }
      else {
        ROS_INFO("%s was not found in the database", i->c_str());
      }
    }

    return coList;
  }

private:
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  ros::AsyncSpinner armSpinner;
  ros::CallbackQueue armQueue;
  ros::CallbackQueue inputQueue;
  ros::Subscriber obsSubscriber;
  ros::Subscriber commSubscriber;
  long lastCommandTime;
  ros::Subscriber jointsSubscriber;

  tf2_ros::Buffer tfBuf;
  tf2_ros::TransformListener tfListener;

  ros::Publisher statusPublisher;
  ros::Publisher camXPublisher;
  geometry_msgs::TransformStamped camXform;
  ros::Timer pubTimer;

  ActionState state;
  std::string failureReason;
  bool armHomeState;

  WorldObjects world;
  ObjectDatabase objData;
  ArmController arm;
  bool gripperClosed;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_motion_server");
    MotionServer ms;
    ms.start();

    ROS_INFO("Starting up the MAIN spinner");
    ros::AsyncSpinner mainSpin(4);
    mainSpin.start();
    ros::waitForShutdown();

    return 0;
}
