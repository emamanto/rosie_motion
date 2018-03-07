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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>

#include "moveit_msgs/CollisionObject.h"
#include "rosie_msgs/RobotCommand.h"
#include "rosie_msgs/RobotAction.h"
#include "gazebo_msgs/ModelStates.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"

class MotionServer
{
public:
  typedef int axis;
  static const axis X_AXIS = 0;
  static const axis Y_AXIS = 1;
  static const axis Z_AXIS = 2;

  typedef std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan_vector;
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

  MotionServer(bool humanCheck=true) : tfBuf(),
                                       tfListener(tfBuf),
                                       state(WAIT),
                                       armHomeState(true),
                                       failureReason("none"),
                                       numRetries(3),
                                       lastCommandTime(0),
                                       grabbedObject("none"),
                                       checkPlans(humanCheck),
                                       gripperClosed(false),
                                       fingerToWrist(-0.16645, 0, 0),
                                       approachOffset(0.08, 0, 0),
                                       grabMotion(0.09, 0, 0),
                                       dropMotion(0.08, 0, 0),
                                       pushOffset(0.06),
                                       group("arm"),
                                       gripper("gripper_controller/gripper_action", true)
  {
        if (!n.getParam("/rosie_motion_server/rosie_is_sim", isSimRobot)) {
          ROS_INFO("RosieMotionServer is missing rosie_is_sim, assuming real robot.");
        }
        else if (isSimRobot == true) {
          ROS_INFO("RosieMotionServer is expecting a simulated robot.");
        }
        else {
          ROS_INFO("RosieMotionServer is expecting a real robot.");
        }

        if (!n.getParam("/rosie_motion_server/human_check", checkPlans)) {
          ROS_INFO("RosieMotionServer is missing human_check parameter, keeping checks on.");
        }
        else if (checkPlans == false) {
          ROS_INFO("RosieMotionServer human checks on motion planning are OFF.");
        }
        else {
          ROS_INFO("RosieMotionServer human checks on motion planning are on.");
        }
        ros::param::set("/move_group/trajectory_execution/allowed_start_tolerance", 0.0);

        group.setMaxVelocityScalingFactor(0.4);

        obsSubscriber = n.subscribe("gazebo/model_states", 10,
                                    &MotionServer::obsCallback, this);
        commSubscriber = n.subscribe("rosie_arm_commands", 10,
                                     &MotionServer::commandCallback, this);
        jointsSubscriber = n.subscribe("joint_states", 10,
                                     &MotionServer::jointCallback, this);

        statusPublisher = n.advertise<rosie_msgs::RobotAction>("rosie_arm_status", 10);
        goalPublisher = n.advertise<geometry_msgs::PoseStamped>("rosie_grasp_target", 10);
        camXPublisher = n.advertise<geometry_msgs::TransformStamped>("rosie_camera", 10);

        gripper.waitForServer();

        graspGoal.pose.position.x = 0;
        graspGoal.pose.position.y = 0;
        graspGoal.pose.position.z = 0;
        graspGoal.pose.orientation.w = 1.0;
        graspGoal.header.frame_id = group.getPlanningFrame();

        approachAngles.push_back(M_PI/2.0);
        approachAngles.push_back(M_PI/3.0);
        approachAngles.push_back(M_PI/4.0);
        approachAngles.push_back(2.0*M_PI/3.0);

        pubTimer = n.createTimer(ros::Duration(0.1),
                                 &MotionServer::publishStatus, this);

        closeGripper();

        shape_msgs::SolidPrimitive glassCup;
        glassCup.type = glassCup.CYLINDER;
        glassCup.dimensions.resize(2);
        glassCup.dimensions[0] = 0.09;
        glassCup.dimensions[1] = 0.04;
        collisionModels.insert(std::pair<std::string,
                               shape_msgs::SolidPrimitive>("cup_glass",
                                                           glassCup));

        shape_msgs::SolidPrimitive coke;
        coke.type = coke.CYLINDER;
        coke.dimensions.resize(2);
        coke.dimensions[0] = 0.12;
        coke.dimensions[1] = 0.035;
        collisionModels.insert(std::pair<std::string,
                               shape_msgs::SolidPrimitive>("coca_cola",
                                                           coke));

        for (int i = 3; i <= 13; i += 2) {
          shape_msgs::SolidPrimitive block;
          block.type = block.BOX;
          block.dimensions.resize(3);
          block.dimensions[0] = (double)i/100;
          block.dimensions[1] = (double)i/100;
          block.dimensions[2] = (double)i/100;
          std::string name = "cube" + std::to_string(i) + "cm";
          collisionModels.insert(std::pair<std::string,
                                 shape_msgs::SolidPrimitive>(name,
                                                             block));
        }

        ROS_INFO("RosieMotionServer READY!");
  };

  void obsCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(objMutex);

    objectPoses.clear();
    objectRotations.clear();

    for (int i = 0; i < msg->name.size(); i++) {
      geometry_msgs::Pose p = msg->pose[i];

      if (msg->name[i].find("fetch") != std::string::npos) {
        tf2::Vector3 fetchVec;
        tf2::fromMsg(p.position, fetchVec);
        tf2::Quaternion fetchQuat;
        tf2::fromMsg(p.orientation, fetchQuat);
        worldXform = tf2::Transform(fetchQuat, fetchVec).inverse();
        continue;
      }
      tf2::Vector3 v;
      tf2::fromMsg(p.position, v);
      objectPoses.insert(std::pair<std::string, tf2::Vector3>(msg->name[i],
                                                              v));

      tf2::Quaternion quat;
      tf2::fromMsg(p.orientation, quat);
      objectRotations.insert(std::pair<std::string, tf2::Quaternion>(msg->name[i],
                                                                     quat));
    }
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
    if (msg->action.find("GRAB")!=std::string::npos)
      {
        state = GRAB;
        std::string name = msg->action.substr(msg->action.find("=")+1);
        ROS_INFO("Handling pickup command for %s", name.c_str());

        setUpScene();
        handleGrabCommand(name);
      }
    else if (msg->action.find("DROP")!=std::string::npos){
      ROS_INFO("Handling putdown command");
      state = DROP;
      std::vector<float> t = std::vector<float>();
      t.push_back(msg->dest.translation.x);
      t.push_back(msg->dest.translation.y);
      t.push_back(msg->dest.translation.z);
      setUpScene();
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
    else if (msg->action.find("POINT")!=std::string::npos){
      state = POINT;
      std::string num = msg->action.substr(msg->action.find("=")+1);
      ROS_INFO("Handling point command for object %s", num.c_str());

      // Same; need to fix
      int idNum;
      std::stringstream ss(num);
      if (!(ss >> idNum)) {
        ROS_INFO("Invalid object ID number %s", num.c_str());
        return;
      }
      setUpScene();
      handlePointCommand(num);
    }
    else if (msg->action.find("HOME")!=std::string::npos){
      ROS_INFO("Handling home command");
      state = HOME;
      homeArm(false);
      state = WAIT;
    }
    else if (msg->action.find("RESET")!=std::string::npos){
      ROS_INFO("Handling reset command");
      state = HOME;
      homeArm(true);
      state = WAIT;
    }
    else if (msg->action.find("SCENE")!=std::string::npos){
      ROS_INFO("Handling build scene command");
      state = SCENE;
      setUpScene();
      state = WAIT;
    }
    else {
      ROS_INFO("Unknown command %s received", msg->action.c_str());
      failureReason = "unknowncommand";
      state = FAILURE;
    }
  }

  void handleGrabCommand(std::string id)
  {
    boost::lock_guard<boost::mutex> guard(objMutex);

    std::string foundName = "";
    for (std::map<std::string, tf2::Vector3>::iterator i = objectPoses.begin();
         i != objectPoses.end(); i++) {
      if (i->first.find(id) != std::string::npos) {
        foundName = i->first;
        break;
      }
    }

    if (foundName == "") {
      ROS_INFO("%s is not being perceived", id.c_str());

      state = FAILURE;
      failureReason = "planning";
      return;
    }

    tf2::Vector3 objVec = worldXform*objectPoses[foundName];
    tf2::Quaternion objQuat = worldXform*objectRotations[foundName];

    float a = planToGraspPosition(objVec.x(),
                                  objVec.y(),
                                  objVec.z(),
                                  tf2::getYaw(objQuat));

    preferredDropAngle = a;
    if (a == -1) {
      ROS_INFO("Arm not reaching because planning failed");
      failureReason = "planning";
      state = FAILURE;
      return;
    }

    armHomeState = false;
    if (!executeCurrentPlan()) {
      ROS_INFO("Arm returning home because execution failed");
      homeArm(true);
      failureReason = "execution";
      state = FAILURE;
      return;
    }

    ros::Duration(0.5).sleep();
    openGripper();

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(group.getCurrentPose().pose);

    geometry_msgs::Pose gp = waypoints[0];

    tf2::Quaternion qtemp;
    qtemp.setRPY(0.0, a, 0.0);
    tf2::Transform rot = tf2::Transform(qtemp);
    tf2::Vector3 ob = tf2::Vector3(gp.position.x,
                                   gp.position.y,
                                   gp.position.z);
    tf2::Vector3 trans = rot*grabMotion;
    tf2::Vector3 out = ob+trans;

    gp.position.x = out.x();
    gp.position.y = out.y();
    gp.position.z = out.z();

    std::vector<float> fPos = eeFrametoFingertip(gp);
    if (fPos[2] < tableH) {
      gp.position.z += (tableH - fPos[2]);
    }

    waypoints.push_back(gp);
    geometry_msgs::Pose returnto = waypoints[0];

    group.setStartStateToCurrentState();
    moveit_msgs::RobotTrajectory inTraj;
    double frac = group.computeCartesianPath(waypoints,
                                             0.01, 0.0,
                                             inTraj,
                                             false);

    currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
    currentPlan.trajectory_ = inTraj;

    if (frac < 0.9 || !executeCurrentPlan()) {
      ROS_INFO("Arm will return home because execution failed");
      failureReason = "grasping";
      state = FAILURE;
    }

    if (state != FAILURE) {
      closeGripper();
    }

    if (gripperClosed) {
      ROS_INFO("Robot seems to have missed block %s", id.c_str());
      std::stringstream ss;
      ss << id;
      std::vector<std::string> missed;
      missed.push_back(ss.str());
      scene.removeCollisionObjects(missed);
      ros::Duration(0.5).sleep();

      ROS_INFO("Arm will return home because grabbing failed");
      failureReason = "grasping";
      state = FAILURE;
    }

    if (state != FAILURE) {
      std::stringstream ss;
      ss << id;
      std::vector<std::string> allowed;
      allowed.push_back("r_gripper_finger_link");
      allowed.push_back("l_gripper_finger_link");
      allowed.push_back("gripper_link");
      group.attachObject(ss.str(), group.getEndEffectorLink(), allowed);
      grabbedObject = id;

      std::vector<std::string> attached;
      attached.push_back(ss.str());

      scene.removeCollisionObjects(attached);
      ros::Duration(0.5).sleep();
    }

    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(group.getCurrentPose().pose);
    waypoints2.push_back(returnto);

    group.setStartStateToCurrentState();
    moveit_msgs::RobotTrajectory outTraj;
    double frac2 = group.computeCartesianPath(waypoints2,
                                              0.01, 0.0,
                                              outTraj,
                                              false);

    currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
    currentPlan.trajectory_ = outTraj;

    if (frac2 < 0.5 || !executeCurrentPlan() || state == FAILURE) {
      ROS_INFO("Arm returning home because execution failed");
      homeArm(true);
      if (state != FAILURE) failureReason = "execution";
      state = FAILURE;
      return;
    }

    homeArm(true);

    if (gripperClosed) {
      ROS_INFO("Robot seems to have dropped block %s", id.c_str());
      grabbedObject = "none";

      std::stringstream ss;
      ss << id;
      std::vector<std::string> missed;
      missed.push_back(ss.str());
      scene.removeCollisionObjects(missed);
      ros::Duration(0.5).sleep();

      failureReason = "grasping";
      state = FAILURE;
      return;
    }

    if (state == GRAB) state = WAIT;
    ROS_INFO("Arm status is now WAIT");
  }

    void handleDropCommand(std::vector<float> target)
    {
      if (grabbedObject == "none") {
        ROS_INFO("Cannot drop because robot is not holding an object.");
        failureReason = "invaliddrop";
        state = FAILURE;
        return;
      }

      if (target[2] == -1) target[2] = tableH;
      target[2] += grabbedObjSize[2] + 0.01;

      // Try the angle you picked it up at first
      tf2::Quaternion qtemp;
      qtemp.setRPY(0.0, preferredDropAngle, 0.0);
      tf2::Transform pRot = tf2::Transform(qtemp);
      tf2::Vector3 tV = tf2::Vector3(target[0], target[1], target[2]);
      tf2::Vector3 transIn = pRot*approachOffset;

      tf2::Vector3 in = tV-transIn;
      float a = -1;
      if (planToXYZAngleTarget(in.x(), in.y(), in.z(), preferredDropAngle, 0)) {
        a = preferredDropAngle;
      } else {
        a = planToGraspPosition(target[0], target[1], target[2]);
      }
      preferredDropAngle = -1;

      if (a == -1) {
        ROS_INFO("Arm not reaching because planning failed");
        failureReason = "planning";
        state = FAILURE;
        return;
      }

      armHomeState = false;
      if (!executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm(true);
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(group.getCurrentPose().pose);
      geometry_msgs::Pose gp = waypoints[0];

      tf2::Quaternion qtemp2;
      qtemp2.setRPY(0.0, a, 0.0);
      tf2::Transform rot = tf2::Transform(qtemp2);
      tf2::Vector3 ob = tf2::Vector3(gp.position.x,
                                   gp.position.y,
                                   gp.position.z);
      tf2::Vector3 trans = rot*dropMotion;
      tf2::Vector3 out = ob+trans;

      gp.position.x = out.x();
      gp.position.y = out.y();
      gp.position.z = out.z();

      std::vector<float> fPos = eeFrametoFingertip(gp);
      tableH += 0.02;
      if (fPos[2] < tableH) {
        gp.position.z += (tableH - fPos[2]);
      }

      waypoints.push_back(gp);
      geometry_msgs::Pose returnto = waypoints[0];

      group.setStartStateToCurrentState();
      moveit_msgs::RobotTrajectory inTraj;
      double frac = group.computeCartesianPath(waypoints,
                                               0.01, 0.0,
                                               inTraj,
                                               false);

      currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
      currentPlan.trajectory_ = inTraj;

      if (frac < 0.8 || !executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm(true);
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();
      openGripper();

      group.detachObject();

      moveit_msgs::CollisionObject droppedObj;
      droppedObj.header.frame_id = group.getPlanningFrame();

      droppedObj.id = grabbedObject;

      std::vector<std::string> toRem;
      toRem.push_back(grabbedObject);
      scene.removeCollisionObjects(toRem);
      ros::Duration(1.0).sleep();

      geometry_msgs::Pose dropP;
      dropP.position.x = target[0];
      dropP.position.y = target[1];
      dropP.position.z = target[2] - 0.01 - grabbedObjSize[2]/2.0;
      dropP.orientation.w = 1.0;

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = grabbedObjSize[0]+0.01;
      primitive.dimensions[1] = grabbedObjSize[1]+0.01;
      primitive.dimensions[2] = grabbedObjSize[2]+0.01;

      droppedObj.primitives.push_back(primitive);
      droppedObj.primitive_poses.push_back(dropP);
      droppedObj.operation = droppedObj.ADD;

      std::vector<moveit_msgs::CollisionObject> toAdd;
      toAdd.push_back(droppedObj);
      scene.addCollisionObjects(toAdd);

      grabbedObject = "none";
      ros::Duration(0.5).sleep();

      std::vector<geometry_msgs::Pose> waypoints2;
      waypoints2.push_back(group.getCurrentPose().pose);
      waypoints2.push_back(returnto);

      group.setStartStateToCurrentState();
      moveit_msgs::RobotTrajectory outTraj;
      double frac2 = group.computeCartesianPath(waypoints2,
                                                0.01, 0.0,
                                                outTraj,
                                                false);

      currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
      currentPlan.trajectory_ = outTraj;

      if (frac2 < 0.8 || !executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm(true);
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();

      closeGripper();
      homeArm(true);

      if (state == DROP) state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    }

    std::vector<float> rotate2D(std::vector<float>& v, float angle) {
        std::vector<float> rotated;
        rotated.push_back(cos(angle)*v[0] - sin(angle)*v[1]);
        rotated.push_back(sin(angle)*v[0] + cos(angle)*v[1]);
        return rotated;
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
    boost::lock_guard<boost::mutex> guard(objMutex);
    if (objectPoses.find(id) == objectPoses.end()) {
      ROS_INFO("Object ID %s is not being perceived", id.c_str());
      return;
    }

    float a = planToGraspPosition(objectPoses[id].x(),
                                  objectPoses[id].y(),
                                  objectPoses[id].z());

    if (a == -1) {
      ROS_INFO("Arm not pointing because planning failed");
      failureReason = "planning";
      state = FAILURE;
      return;
    }

    armHomeState = false;
    if (!executeCurrentPlan()) {
      ROS_INFO("Arm returning home because execution failed");
      homeArm(true);
      failureReason = "execution";
      state = FAILURE;
      return;
    }

    ros::Duration(1.0).sleep();
    homeArm(true);
    if (state == POINT) state = WAIT;
    ROS_INFO("Arm status is now WAIT");
  }


  float planToGraspPosition(float objx, float objy, float objz) {
    return planToGraspPosition(objx, objy, objz, 0.0);
  }

  float planToGraspPosition(float objx, float objy, float objz, float yaw)
  {
    //ROS_INFO("Planning to a grasp yaw of: %f", yaw);
    float foundAngle = -1;
    int tries = 0;

    while (tries < numRetries) {
      for (std::vector<float>::iterator i = approachAngles.begin();
           i != approachAngles.end(); i++) {
        tf2::Transform rot = tf2::Transform(yawPitchToQuat(yaw, *i));
        tf2::Vector3 ob = tf2::Vector3(objx, objy, objz);
        tf2::Vector3 trans = rot*approachOffset;

        tf2::Vector3 out = ob-trans;
        if (planToXYZAngleTarget(out.x(), out.y(), out.z(), *i, yaw)) {
          foundAngle = *i;
          break;
        }
      }

      if (foundAngle != -1) break;
      tries++;
    }

    return foundAngle;
  }

  void publishStatus(const ros::TimerEvent& e)
  {
    rosie_msgs::RobotAction msg = rosie_msgs::RobotAction();
    msg.utime = ros::Time::now().toNSec();
    msg.action = asToString(state).c_str();
    msg.armHome = armHomeState;
    msg.failure_reason = failureReason;
    msg.obj_id = grabbedObject;
    statusPublisher.publish(msg);
    goalPublisher.publish(graspGoal);

    try {
      camXform = tfBuf.lookupTransform("base_link", "head_camera_rgb_optical_frame",
                                       ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    camXPublisher.publish(camXform);
  }

  void setUpScene()
  {
    std::vector<std::string> known = scene.getKnownObjectNames();
    scene.removeCollisionObjects(known);
    //ROS_INFO("Removing known collision objects");
    ros::Duration(1).sleep();

    boost::lock_guard<boost::mutex> guard(objMutex);
    std::vector<moveit_msgs::CollisionObject> coList;

    for (std::map<std::string, tf2::Vector3>::iterator i = objectPoses.begin();
         i != objectPoses.end(); i++) {
      // Ignore the fetch
      if (i->first == "fetch") continue;

      // Check if the fetch could actually hit this
      tf2::Vector3 objVec(i->second[0],
                          i->second[1],
                          i->second[2]);
      float dist = tf2::tf2Distance(worldXform.getOrigin(), objVec);

      if (dist > 2) {
        ROS_INFO("Object %s is out of reasonable range", i->first.c_str());
        continue;
      }
      else if (i->first == "ground_plane") {
        ROS_INFO("Do something with ground plane");
        continue;
      }
      else if (i->first.find("table") != std::string::npos) {
        ROS_INFO("Do something with table");
        continue;
      }

      moveit_msgs::CollisionObject co;
      co.header.frame_id = group.getPlanningFrame();

      co.id = i->first;
      ROS_INFO("Adding object %s", i->first.c_str());

      geometry_msgs::Pose box_pose;
      tf2::Vector3 fetchCentered = worldXform*objVec;
      box_pose.position.x = fetchCentered.x();
      box_pose.position.y = fetchCentered.y();
      box_pose.position.z = fetchCentered.z();

      tf2::Quaternion objQuat = objectRotations[i->first];
      geometry_msgs::Quaternion q = tf2::toMsg(worldXform*objQuat);
      box_pose.orientation = q;

      shape_msgs::SolidPrimitive primitive;
      bool found = false;
      for (std::map<std::string, shape_msgs::SolidPrimitive>::iterator j =
             collisionModels.begin();
           j != collisionModels.end(); j++) {
        if (i->first.find(j->first) != std::string::npos) {
          found = true;
          ROS_INFO("Found shape under name %s", j->first.c_str());
          primitive = j->second;
          break;
        }
      }
      if (!found) continue;

      co.primitives.push_back(primitive);
      co.primitive_poses.push_back(box_pose);
      co.operation = co.ADD;
      coList.push_back(co);
    }
    scene.addCollisionObjects(coList);
    ros::Duration(2).sleep();
  }

  // Ignores all objects for a last-ditch effort to get home even
  // if arm hits something
  void setUpEmptyScene() {
    std::vector<std::string> known = scene.getKnownObjectNames();
    scene.removeCollisionObjects(known);
    //ROS_INFO("Removing known collision objects");
    ros::Duration(1).sleep();

    std::vector<moveit_msgs::CollisionObject> coList;

    // JUST add the table
    moveit_msgs::CollisionObject planeobj;
    planeobj.header.frame_id = group.getPlanningFrame();
    planeobj.id = "table";

    geometry_msgs::Pose planep;
    planep.position.x = 0.8;
    planep.position.y = 0.0;
    planep.position.z = tableH;
    planep.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.02;

    planeobj.primitives.push_back(primitive);
    planeobj.primitive_poses.push_back(planep);
    planeobj.operation = planeobj.ADD;
    coList.push_back(planeobj);

    scene.addCollisionObjects(coList);
    ros::Duration(1).sleep();
  }

  bool safetyCheck()
  {
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

  void homeArm(bool forceHome = false)
  {
    ros::Duration(0.5).sleep();

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

    int tries = 0;
    std::string failure = "";
    moveit::planning_interface::MoveItErrorCode success;
    while(!success && tries < numRetries) {
      tries++;
      group.setStartStateToCurrentState();
      success = group.plan(homePlan);
      if (!success) {
        failure = "planning";
        continue;
      }

      if (!safetyCheck()) {
        failureReason = "safety";
        state = FAILURE;
        return;
      }

      moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(homePlan);
      success = moveSuccess;
      if (!moveSuccess) {
        failure = "execution";
      }
    }

    if (!success && forceHome) {
      ROS_INFO("Homing arm failed with blocks in the way, attempting to force home");
      setUpEmptyScene();

      int forceCount = 0;
      while (!success && forceCount < 10) {
        group.setStartStateToCurrentState();
        success = group.plan(homePlan);
        if (!success) {
          failure = "planning";
        }

        if (!safetyCheck()) {
          failureReason = "safety";
          state = FAILURE;
          return;
        }

        moveit::planning_interface::MoveItErrorCode fhSuccess = group.execute(homePlan);
        success = fhSuccess;
        if (!fhSuccess) {
          failure = "execution";
        }
        forceCount++;
      }
    }

    if (!success) {
      ROS_INFO("Homing arm failed.");
      return;
    }

    // This should ONLY get set back to true HERE after success
    armHomeState = true;
  }

  void closeGripper()
  {
    setGripperTo(0.0);
  }

  void openGripper()
  {
    setGripperTo(0.1);
  }

  void setGripperTo(float m)
  {
    control_msgs::GripperCommandGoal gripperGoal;
    gripperGoal.command.max_effort = 0.0;
    gripperGoal.command.position = m;

    gripper.sendGoal(gripperGoal);
    gripper.waitForResult(ros::Duration(2.0));
  }

  // Estimate whether the gripper can reach a position on the tabletop
  bool tooFar(float x, float y, bool gripperDown)
  {
    if (gripperDown)
      return (sqrt(pow(x, 2) + pow(y, 2)) > 0.82);
    else
      return (sqrt(pow(x, 2) + pow(y, 2)) > 0.95);
  }

  bool tooFar(std::vector<float> loc, bool gripperDown)
  {
    return tooFar(loc[0], loc[1], gripperDown);
  }

  geometry_msgs::Pose xyzypTargetToPoseMsg(float x, float y, float z,
                                           float yaw, float pitch)
  {
    tf2::Quaternion q = yawPitchToQuat(yaw, pitch);
    geometry_msgs::Pose p = geometry_msgs::Pose();
    p.orientation = tf2::toMsg(q);

    std::vector<float> eeTarg = fingertipToEEFrame(x, y, z, q);

    p.position.x = eeTarg[0];
    p.position.y = eeTarg[1];
    p.position.z = eeTarg[2];

    return p;
  }


  bool planToXYZQuaternionTarget(float x, float y, float z, tf2::Quaternion q)
  {
    group.setStartStateToCurrentState();
    geometry_msgs::Pose target = geometry_msgs::Pose();
    target.orientation = tf2::toMsg(q);

    std::vector<float> targ = fingertipToEEFrame(x, y, z, q);
    target.position.x = targ[0];
    target.position.y = targ[1];
    target.position.z = targ[2];

    geometry_msgs::Pose fingerTarget = geometry_msgs::Pose();
    fingerTarget.orientation = tf2::toMsg(q);
    fingerTarget.position.x = x;
    fingerTarget.position.y = y;
    fingerTarget.position.z = z;

    graspGoal.pose = fingerTarget;
    graspGoal.header.frame_id = group.getPlanningFrame();
    ros::Duration(1.0).sleep();

    group.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan xyzPlan;

    moveit::planning_interface::MoveItErrorCode success = group.plan(xyzPlan);
    // To stop it thinking it's successful if postprocessing is the problem
    if (xyzPlan.trajectory_.joint_trajectory.points.empty() &&
        xyzPlan.trajectory_.multi_dof_joint_trajectory.points.empty()) {
      success = false;
    }

    if (!success) {
      currentPlan = moveit::planning_interface::MoveGroupInterface::Plan();
      return false;
    }

    currentPlan = xyzPlan;
    return true;
  }

  bool planToXYZAngleTarget(float x, float y, float z, float pitch, float yaw)
  {
    tf2::Quaternion q = yawPitchToQuat(yaw, pitch);
    return planToXYZQuaternionTarget(x, y, z, q);
  }

  bool executeCurrentPlan()
  {
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

  std::vector<float> eeFrametoFingertip(geometry_msgs::Pose p)
  {
    tf2::Transform rotationX;
    tf2::fromMsg(p, rotationX);
    rotationX.setOrigin(tf2::Vector3(0, 0, 0));

    tf2::Vector3 trans = rotationX*tf2::Vector3(-fingerToWrist[0],
                                                -fingerToWrist[1],
                                                -fingerToWrist[2]);

    std::vector<float> res;
    res.push_back(p.position.x + trans.x());
    res.push_back(p.position.y + trans.y());
    res.push_back(p.position.z + trans.z());

    return res;
  }

  std::vector<float> fingertipToEEFrame(float fx, float fy, float fz,
                                        float hr, float hp, float hy)
  {
    std::vector<float> finger;
    finger.push_back(fx);
    finger.push_back(fy);
    finger.push_back(fz);

    std::vector<float> ang;
    ang.push_back(hr);
    ang.push_back(hp);
    ang.push_back(hy);

    return fingertipToEEFrame(finger, ang);
  }

  std::vector<float> fingertipToEEFrame(float fx, float fy, float fz,
                                        tf2::Quaternion q)
  {
    std::vector<float> finger;
    finger.push_back(fx);
    finger.push_back(fy);
    finger.push_back(fz);

    return fingertipToEEFrame(finger, q);
  }

  std::vector<float> fingertipToEEFrame(std::vector<float> fingertip,
                                        tf2::Quaternion q)
  {
    tf2::Transform rot = tf2::Transform(q);
    tf2::Vector3 ft = tf2::Vector3(fingertip[0],
                                   fingertip[1],
                                   fingertip[2]);
    tf2::Vector3 trans = rot*tf2::Vector3(fingerToWrist[0],
                                          fingerToWrist[1],
                                          fingerToWrist[2]);

    tf2::Vector3 out = ft+trans;

    std::vector<float> res;
    res.push_back(out.x());
    res.push_back(out.y());
    res.push_back(out.z());

    return res;
  }

  std::vector<float> fingertipToEEFrame(std::vector<float> fingertip,
                                        std::vector<float> handRPY)
  {
    tf2::Quaternion q;
    q.setRPY(handRPY[0], handRPY[1], handRPY[2]);
    return fingertipToEEFrame(fingertip, q);
  }

private:
  ros::NodeHandle n;
  ros::Subscriber obsSubscriber;
  ros::Subscriber commSubscriber;
  ros::Subscriber jointsSubscriber;

  tf2_ros::Buffer tfBuf;
  tf2_ros::TransformListener tfListener;

  ros::Publisher statusPublisher;
  ros::Publisher goalPublisher;
  ros::Publisher camXPublisher;
  geometry_msgs::PoseStamped graspGoal;
  geometry_msgs::TransformStamped camXform;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
  ros::Timer pubTimer;

  ActionState state;
  bool armHomeState;
  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  std::string failureReason;
  int numRetries;
  long lastCommandTime;
  std::string grabbedObject;
  std::vector<float> grabbedObjSize;
  bool gripperClosed;
  bool checkPlans;
  bool isSimRobot;
  tf2::Vector3 fingerToWrist;

  // Grasp position variations
  std::vector<float> approachAngles;
  float preferredDropAngle;
  tf2::Vector3 approachOffset;
  tf2::Vector3 grabMotion;
  tf2::Vector3 dropMotion;
  float pushOffset;

  moveit::planning_interface::MoveGroupInterface group;
  moveit::planning_interface::PlanningSceneInterface scene;

  std::map<std::string, tf2::Vector3> objectPoses;
  std::map<std::string, tf2::Quaternion> objectRotations;
  tf2::Transform worldXform;
  float tableH;
  boost::mutex objMutex;

  std::map<std::string, shape_msgs::SolidPrimitive> collisionModels;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_motion_server");
    MotionServer ms;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
