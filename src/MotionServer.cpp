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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

#include "moveit_msgs/CollisionObject.h"
#include "rosie_msgs/RobotCommand.h"
#include "rosie_msgs/RobotAction.h"
#include "rosie_msgs/Observations.h"
#include "rosie_msgs/ObjectData.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"

class MotionServer
{
public:
    typedef int axis;
    static const axis X_AXIS = 0;
    static const axis Y_AXIS = 1;
    static const axis Z_AXIS = 2;

    typedef std::vector<moveit::planning_interface::MoveGroup::Plan> plan_vector;
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

  MotionServer(bool humanCheck=true) : state(WAIT),
                                       failureReason("none"),
                                       numRetries(3),
                                       lastCommandTime(0),
                                       grabbedObject(-1),
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

        obsSubscriber = n.subscribe("rosie_observations", 10,
                                    &MotionServer::obsCallback, this);
        commSubscriber = n.subscribe("rosie_arm_commands", 10,
                                     &MotionServer::commandCallback, this);
        jointsSubscriber = n.subscribe("joint_states", 10,
                                     &MotionServer::jointCallback, this);

        statusPublisher = n.advertise<rosie_msgs::RobotAction>("rosie_arm_status", 10);
        goalPublisher = n.advertise<geometry_msgs::PoseStamped>("rosie_grasp_target", 10);

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
        ROS_INFO("RosieMotionServer READY!");
    };

    void obsCallback(const rosie_msgs::Observations::ConstPtr& msg)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);

        objectPoses.clear();
        objectSizes.clear();
        objectRotations.clear();

        currentTable = msg->table;

        for (std::vector<rosie_msgs::ObjectData>::const_iterator i = msg->observations.begin();
             i != msg->observations.end(); i++) {
            std::vector<float> pos = std::vector<float>();
            pos.push_back(i->bbox_xyzrpy.translation.x);
            pos.push_back(i->bbox_xyzrpy.translation.y);
            pos.push_back(i->bbox_xyzrpy.translation.z);
            objectPoses.insert(std::pair<int, std::vector<float> >(i->obj_id, pos));

            tf::Quaternion quat;
            tf::quaternionMsgToTF(i->bbox_xyzrpy.rotation, quat);
            objectRotations.insert(std::pair<int, tf::Quaternion>(i->obj_id, quat));

            std::vector<float> dim = std::vector<float>();
            dim.push_back(i->bbox_dim.x);
            dim.push_back(i->bbox_dim.y);
            dim.push_back(i->bbox_dim.z);
            objectSizes.insert(std::pair<int, std::vector<float> >(i->obj_id, dim));
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
            std::string num = msg->action.substr(msg->action.find("=")+1);
            ROS_INFO("Handling pickup command for object %s", num.c_str());

            int idNum;
            std::stringstream ss(num);
            if (!(ss >> idNum)) {
                ROS_INFO("Invalid object ID number %s", num.c_str());
                return;
            }

            setUpScene();
            handleGrabCommand(idNum);
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
        else if (msg->action.find("PUSH")!=std::string::npos){
            state = PUSH;
            std::string num = msg->action.substr(msg->action.find("=")+1);
            ROS_INFO("Handling push command for object %s", num.c_str());

            int idNum;
            std::stringstream ss(num);
            if (!(ss >> idNum)) {
                ROS_INFO("Invalid object ID number %s", num.c_str());
                return;
            }

            std::vector<float> t = std::vector<float>();
            t.push_back(msg->dest.translation.x);
            t.push_back(msg->dest.translation.y);
            t.push_back(msg->dest.translation.z);

            setUpScene();
            handlePushCommand(idNum, t);
        }
        else if (msg->action.find("POINT")!=std::string::npos){
            state = POINT;
            std::string num = msg->action.substr(msg->action.find("=")+1);
            ROS_INFO("Handling point command for object %s", num.c_str());

            int idNum;
            std::stringstream ss(num);
            if (!(ss >> idNum)) {
                ROS_INFO("Invalid object ID number %s", num.c_str());
                return;
            }
            setUpScene();
            handlePointCommand(idNum);
        }
        else if (msg->action.find("HOME")!=std::string::npos){
            ROS_INFO("Handling home command");
            state = HOME;
            homeArm();
            state = WAIT;
        }
        else if (msg->action.find("RESET")!=std::string::npos){
            ROS_INFO("Handling reset command");
            state = HOME;
            homeArm();
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

    void handleGrabCommand(int id)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);
        if (objectSizes.find(id) == objectSizes.end() ||
            objectPoses.find(id) == objectSizes.end()) {
          ROS_INFO("Object ID %d is not being perceived", id);

          state = FAILURE;
          return;
        }

        // This only works for objects that can be picked up like squares!!!
        float yaw = tf::getYaw(objectRotations[id]);
        if (fabs(yaw + M_PI/2.0) < fabs(yaw)) yaw += M_PI/2.0;
        if (fabs(yaw - M_PI/2.0) < fabs(yaw)) yaw -= M_PI/2.0;

        float a = planToGraspPosition(objectPoses[id][0],
                                      objectPoses[id][1],
                                      objectPoses[id][2] + objectSizes[id][2]/2.0,
                                      yaw);

        preferredDropAngle = a;
        if (a == -1) {
          ROS_INFO("Arm returning home because planning failed");
          homeArm();
          failureReason = "planning";
          state = FAILURE;
          return;
        }
        if (!executeCurrentPlan()) {
          ROS_INFO("Arm returning home because execution failed");
          homeArm();
          failureReason = "execution";
          state = FAILURE;
          return;
        }

        ros::Duration(0.5).sleep();
        openGripper();

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(group.getCurrentPose().pose);

        geometry_msgs::Pose gp = waypoints[0];

        tf::Transform rot = tf::Transform(tf::createQuaternionFromRPY(0.0,
                                                                      a,
                                                                      0.0));
        tf::Vector3 ob = tf::Vector3(gp.position.x,
                                     gp.position.y,
                                     gp.position.z);
        tf::Vector3 trans = rot*grabMotion;
        tf::Vector3 out = ob+trans;

        gp.position.x = out.x();
        gp.position.y = out.y();
        gp.position.z = out.z();

        std::vector<float> fPos = eeFrametoFingertip(gp);
        float tableH = ((currentTable[3] + currentTable[0]*objectPoses[id][0] +
                         currentTable[1]*objectPoses[id][1]) / -currentTable[2]) + 0.04;
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

        currentPlan = moveit::planning_interface::MoveGroup::Plan();
        currentPlan.trajectory_ = inTraj;

        if (frac < 0.9 || !executeCurrentPlan()) {
          ROS_INFO("Arm returning home because execution failed");
          homeArm();
          failureReason = "grasping";
          state = FAILURE;
          return;
        }

        closeGripper();

        if (gripperClosed) {
          ROS_INFO("Robot seems to have missed block %d", id);
          std::stringstream ss;
          ss << id;
          std::vector<std::string> missed;
          missed.push_back(ss.str());
          scene.removeCollisionObjects(missed);
          ros::Duration(0.5).sleep();

          ROS_INFO("Arm returning home because grabbing failed");
          homeArm();
          failureReason = "grasping";
          state = FAILURE;
          return;
        }

        std::stringstream ss;
        ss << id;
        std::vector<std::string> allowed;
        allowed.push_back("r_gripper_finger_link");
        allowed.push_back("l_gripper_finger_link");
        allowed.push_back("gripper_link");
        group.attachObject(ss.str(), group.getEndEffectorLink(), allowed);
        grabbedObject = id;
        grabbedObjSize = objectSizes[id];

        std::vector<std::string> attached;
        attached.push_back(ss.str());

        scene.removeCollisionObjects(attached);
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

        currentPlan = moveit::planning_interface::MoveGroup::Plan();
        currentPlan.trajectory_ = outTraj;

        if (frac2 < 0.8 || !executeCurrentPlan()) {
          ROS_INFO("Arm returning home because execution failed");
          homeArm();
          failureReason = "execution";
          state = FAILURE;
          return;
        }

        homeArm();

        if (gripperClosed) {
          ROS_INFO("Robot seems to have dropped block %d", id);
          grabbedObject = -1;

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
      if (grabbedObject == -1) {
        ROS_INFO("Cannot drop because robot is not holding an object.");
        failureReason = "invaliddrop";
        state = FAILURE;
        return;
      }

      float tableH = ((currentTable[3] + currentTable[0]*target[0] +
                       currentTable[1]*target[1]) / -currentTable[2]) + 0.02;

      if (target[2] == -1) target[2] = tableH;
      target[2] += grabbedObjSize[2] + 0.01;

      // Try the angle you picked it up at first
      tf::Transform pRot = tf::Transform(tf::createQuaternionFromRPY(0.0,
                                                                    preferredDropAngle,
                                                                    0.0));
      tf::Vector3 tV = tf::Vector3(target[0], target[1], target[2]);
      tf::Vector3 transIn = pRot*approachOffset;

      tf::Vector3 in = tV-transIn;
      float a = -1;
      if (planToXYZAngleTarget(in.x(), in.y(), in.z(), preferredDropAngle, 0)) {
        a = preferredDropAngle;
      } else {
        a = planToGraspPosition(target[0], target[1], target[2]);
      }
      preferredDropAngle = -1;

      if (a == -1) {
        ROS_INFO("Arm returning home because planning failed");
        homeArm();
        failureReason = "planning";
        state = FAILURE;
        return;
      }

      if (!executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm();
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(group.getCurrentPose().pose);
      geometry_msgs::Pose gp = waypoints[0];
      tf::Transform rot = tf::Transform(tf::createQuaternionFromRPY(0.0,
                                                                    a,
                                                                    0.0));
      tf::Vector3 ob = tf::Vector3(gp.position.x,
                                   gp.position.y,
                                   gp.position.z);
      tf::Vector3 trans = rot*dropMotion;
      tf::Vector3 out = ob+trans;

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

      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      currentPlan.trajectory_ = inTraj;

      if (frac < 0.8 || !executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm();
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();
      openGripper();

      group.detachObject();

      moveit_msgs::CollisionObject droppedObj;
      droppedObj.header.frame_id = group.getPlanningFrame();

      std::stringstream ss;
      ss << grabbedObject;
      droppedObj.id = ss.str();

      std::vector<std::string> toRem;
      toRem.push_back(ss.str());
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

      grabbedObject = -1;
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

      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      currentPlan.trajectory_ = outTraj;

      if (frac2 < 0.8 || !executeCurrentPlan()) {
        ROS_INFO("Arm returning home because execution failed");
        homeArm();
        failureReason = "execution";
        state = FAILURE;
        return;
      }

      ros::Duration(0.5).sleep();

      closeGripper();
      homeArm();

      if (state == DROP) state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    }

    std::vector<float> rotate2D(std::vector<float>& v, float angle) {
        std::vector<float> rotated;
        rotated.push_back(cos(angle)*v[0] - sin(angle)*v[1]);
        rotated.push_back(sin(angle)*v[0] + cos(angle)*v[1]);
        return rotated;
    }

    void handlePushCommand(int id, std::vector<float> pV)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);
        if (objectSizes.find(id) == objectSizes.end() ||
            objectPoses.find(id) == objectSizes.end()) {
          boost::lock_guard<boost::mutex> guard(objMutex);
          ROS_INFO("Object ID %d is not being perceived", id);
          failureReason = "invalidpush";
          state = FAILURE;
          return;
        }


        if (pV[0] == 0 && pV[1] == 0) {
          ROS_INFO("No need to push; not planning motion");
          failureReason = "";
          state = WAIT;
          return;
        }

        float objYaw = tf::getYaw(objectRotations[id]);
        std::vector<float> chosenPush;

        // Plans the reach and in/out motions at once
        // Prefer X push
        plan_vector steps;
        if (pV[2] == X_AXIS || (pV[2] == -1 && fabs(pV[0]) <= fabs(pV[1]))) {
          // Try X axis push
          steps = planPush(id, pV[0], X_AXIS);
          if (!steps.empty()) {
            chosenPush.push_back(pV[0]);
            chosenPush.push_back(0.0);
          }
          // Try Y axis push if X failed and it's available
          else if (steps.empty() && pV[1] != 0) {
            steps = planPush(id, pV[1], Y_AXIS);
            if (!steps.empty()) {
              chosenPush.push_back(0.0);
              chosenPush.push_back(pV[1]);
            }
          }
        }
        // Prefer Y push
        if (pV[2] == Y_AXIS || (pV[2] == -1 && fabs(pV[1]) < fabs(pV[0]))) {
          // Try Y axis push
          steps = planPush(id, pV[1], Y_AXIS);
          if (!steps.empty()) {
            chosenPush.push_back(0.0);
            chosenPush.push_back(pV[1]);
          }
          // Try X axis push if Y failed and it's available
          else if (steps.empty() && pV[0] != 0) {
            steps = planPush(id, pV[0], X_AXIS);
            if (!steps.empty()) {
              chosenPush.push_back(pV[0]);
              chosenPush.push_back(0.0);
            }
          }
        }

        // Total failure
        if (steps.empty()) {
          ROS_INFO("Could not find a plan to push");
          failureReason = "planning";
          state = FAILURE;
          return;
        }

        // Otherwise, execute the steps we found

        setGripperTo(0.02);
        for (plan_vector::iterator i = steps.begin(); i != steps.end(); i++) {
          ros::Duration(0.5).sleep();
          currentPlan = *i;
          std::vector<double> j = group.getCurrentJointValues();
          ROS_INFO("Starting next exec at: %f, %f, %f, %f, %f, %f, %f", j[0],
                   j[1],
                   j[2],
                   j[3],
                   j[4],
                   j[5],
                   j[6]);

          if (!executeCurrentPlan()) {
            ROS_INFO("Arm returning home because execution failed");
            homeArm();
            failureReason = "execution";
            state = FAILURE;
            return;
          }
        }

        // Remove block from collision map
        std::stringstream ss;
        ss << id;
        std::vector<std::string> pushed;
        pushed.push_back(ss.str());
        scene.removeCollisionObjects(pushed);
        ros::Duration(0.5).sleep();

        // Re-add block to collision map
        moveit_msgs::CollisionObject pushedObj;
        pushedObj.header.frame_id = group.getPlanningFrame();

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = objectSizes[id][0] + 0.01;
        primitive.dimensions[1] = objectSizes[id][1] + 0.01;
        primitive.dimensions[2] = objectSizes[id][2] + 0.01;

        std::vector<float> trans = rotate2D(chosenPush, objYaw);
        geometry_msgs::Pose pushP;
        pushP.position.x = objectPoses[id][0] + trans[0];
        pushP.position.y = objectPoses[id][1] + trans[1];
        pushP.position.z = objectPoses[id][2];
        pushP.orientation.w = objectRotations[id].w();
        pushP.orientation.x = objectRotations[id].x();
        pushP.orientation.y = objectRotations[id].y();
        pushP.orientation.z = objectRotations[id].z();

        pushedObj.primitives.push_back(primitive);
        pushedObj.primitive_poses.push_back(pushP);
        pushedObj.operation = pushedObj.ADD;

        std::vector<moveit_msgs::CollisionObject> toAdd;
        toAdd.push_back(pushedObj);
        scene.addCollisionObjects(toAdd);
        ros::Duration(0.5).sleep();

        homeArm();
        if (state == PUSH) state = WAIT;
        ROS_INFO("Arm status is now WAIT");
    }

  plan_vector planPush(int id, float dist, axis a)
  {
    float x = objectPoses[id][0];
    float y = objectPoses[id][1];
    float z = objectPoses[id][2] + objectSizes[id][2]/2.0 - 0.01;
    float yaw = tf::getYaw(objectRotations[id]);

    plan_vector plans;

    // Plan to the offset point
    std::vector<float> offsetV;
    if (a == Y_AXIS) offsetV.push_back(0);
    if (dist < 0) {
      offsetV.push_back(objectSizes[id][1]/2.0 + pushOffset + 0.01);
    }
    else if (dist > 0) {
      offsetV.push_back(-(objectSizes[id][1]/2.0 + pushOffset + 0.01));
    }
    if (a == X_AXIS) offsetV.push_back(0);

    std::vector<float> rotatedOffset = rotate2D(offsetV, yaw);
    x += rotatedOffset[0];
    y += rotatedOffset[1];
    z += 0.04;

    float pushYaw = yaw;
    if (a == Y_AXIS) pushYaw += M_PI/2.0;
    while (pushYaw > M_PI) pushYaw -= M_PI;

    // Retry the original reach several times
    bool success = false;
    int tries = 0;
    while (!success && tries < numRetries) {
      success = planToXYZAngleTarget(x, y, z, M_PI/2.0, pushYaw);
      if (!success) {
        success = planToXYZAngleTarget(x, y, z, M_PI/2.0, pushYaw+M_PI);
        if (success) pushYaw += M_PI;
      }
      if (!success) {
        success = planToXYZAngleTarget(x, y, z, M_PI/2.0, pushYaw-M_PI);
        if (success) pushYaw -= M_PI;
      }
      tries++;
    }

    if (success) {
      ROS_INFO("Plan for initial reach found");
      plans.push_back(currentPlan);
    }
    else {
      ROS_INFO("Could not find a plan to reach push position");
      plans.clear();
      return plans;
    }

    // Setup motion to touch the side of the block
    std::vector<float> setupV;
    if (a == Y_AXIS) setupV.push_back(0);
    if (dist < 0) {
      setupV.push_back(-(pushOffset - 0.01));
    }
    else {
      setupV.push_back(pushOffset - 0.01);
    }
    if (a == X_AXIS) setupV.push_back(0);

    robot_state::RobotState setupStartState(*group.getCurrentState());
    setupStartState.setJointGroupPositions(group.getName(), currentPlan.trajectory_.joint_trajectory.points.back().positions);
    group.setStartState(setupStartState);

    std::vector<float> rotatedSetup = rotate2D(setupV, yaw);
    moveit_msgs::RobotTrajectory setupTraj;
    std::vector<geometry_msgs::Pose> setupWaypoints;
    //setupWaypoints.push_back(xyzypTargetToPoseMsg(x, y, z, pushYaw, M_PI/2.0));

    geometry_msgs::Pose tp = setupWaypoints[0];
    tp.position.x += rotatedSetup[0];
    tp.position.y += rotatedSetup[1];
    tp.position.z -= 0.04;

    std::vector<float> fPos = eeFrametoFingertip(tp);
    float tableH = ((currentTable[3] + currentTable[0]*objectPoses[id][0] +
                     currentTable[1]*objectPoses[id][1]) / -currentTable[2]) + 0.04;
    if (fPos[2] < tableH) {
      tp.position.z += (tableH - fPos[2]);
    }
    setupWaypoints.push_back(tp);
    graspGoal.pose = tp;
    graspGoal.header.frame_id = group.getPlanningFrame();

    double sFrac = group.computeCartesianPath(setupWaypoints,
                                              0.01, 0.0,
                                              setupTraj,
                                              false);

    if (sFrac > 0.9) {
      ROS_INFO("Plan for setup motion found");
      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      currentPlan.trajectory_ = setupTraj;
      plans.push_back(currentPlan);

      std::vector<double> tmp = currentPlan.trajectory_.joint_trajectory.points[0].positions;
      ROS_INFO("First cartesian: %f, %f, %f, %f, %f, %f, %f", tmp[0],
             tmp[1],
             tmp[2],
             tmp[3],
             tmp[4],
             tmp[5],
             tmp[6]);

    }
    else {
      ROS_INFO("Could not find a plan for setup motion");
      plans.clear();
      return plans;
    }

    // Actual pushing motion
    std::vector<float> pushV;
    if (a == Y_AXIS) pushV.push_back(0);
    pushV.push_back(dist);
    if (a == X_AXIS) pushV.push_back(0);

    robot_state::RobotState pushStartState(*group.getCurrentState());
    pushStartState.setJointGroupPositions(group.getName(), currentPlan.trajectory_.joint_trajectory.points.back().positions);
    group.setStartState(pushStartState);

    std::vector<float> rotatedPush = rotate2D(pushV, yaw);
    moveit_msgs::RobotTrajectory pushTraj;
    std::vector<geometry_msgs::Pose> pushWaypoints;
    pushWaypoints.push_back(setupWaypoints.back());

    geometry_msgs::Pose pp = pushWaypoints[0];
    geometry_msgs::Pose returnto = pushWaypoints[0];

    pp.position.x += rotatedPush[0];
    pp.position.y += rotatedPush[1];
    pushWaypoints.push_back(pp);

    double pFrac = group.computeCartesianPath(pushWaypoints,
                                              0.01, 0.0,
                                              pushTraj,
                                              false);

    currentPlan = moveit::planning_interface::MoveGroup::Plan();
    currentPlan.trajectory_ = pushTraj;

    if (pFrac > 0.9) {
      ROS_INFO("Plan for push motion found");
      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      currentPlan.trajectory_ = setupTraj;
      plans.push_back(currentPlan);
    }
    else {
      ROS_INFO("Could not find a plan for push motion");
      plans.clear();
      return plans;
    }

    robot_state::RobotState outStartState(*group.getCurrentState());
    outStartState.setJointGroupPositions(group.getName(), currentPlan.trajectory_.joint_trajectory.points.back().positions);
    group.setStartState(outStartState);

    moveit_msgs::RobotTrajectory outTraj;
    std::vector<geometry_msgs::Pose> outWaypoints;
    outWaypoints.push_back(pushWaypoints.back());

    if (fabs(dist) < 0.03) {
      ROS_INFO("Short push, returning to start point");
      returnto.position.z += 0.03;
      outWaypoints.push_back(returnto);
    }
    else {
      ROS_INFO("Long push, calculating a shorter return vector");
      geometry_msgs::Pose tp = outWaypoints[0];
      std::vector<float> out = rotatedPush;
      float len = sqrt(pow(out[0], 2) + pow(out[1], 2));
      out[0] = 0.03 * (out[0] / len);
      out[1] = 0.03 * (out[1] / len);
      tp.position.x -= out[0];
      tp.position.y -= out[1];
      tp.position.z += 0.03;
      setupWaypoints.push_back(tp);
    }

    double oFrac = group.computeCartesianPath(outWaypoints,
                                              0.01, 0.0,
                                              outTraj,
                                              false);

    currentPlan = moveit::planning_interface::MoveGroup::Plan();

    currentPlan.trajectory_ = outTraj;
    if (pFrac > 0.9) {
      ROS_INFO("Plan for outward motion found");
      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      currentPlan.trajectory_ = setupTraj;
      plans.push_back(currentPlan);
    }
    else {
      ROS_INFO("Could not find a plan for outward motion");
      plans.clear();
      return plans;
    }

    return plans;
  }

    tf::Quaternion yawPitchToQuat(float yaw, float pitch)
    {
      std::vector<float> res;

      tf::Transform first = tf::Transform(tf::createQuaternionFromRPY(0, 0, yaw));
      tf::Transform second = tf::Transform(tf::createQuaternionFromRPY(0, pitch, 0));

      tf::Transform t = first*second;

      return t.getRotation();
    }

    void handlePointCommand(int id)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);
        if (objectSizes.find(id) == objectSizes.end()||
            objectPoses.find(id) == objectSizes.end()) {
          ROS_INFO("Object ID %d is not being perceived", id);
          return;
        }

        float a = planToGraspPosition(objectPoses[id][0],
                                      objectPoses[id][1],
                                      objectPoses[id][2] + objectSizes[id][2]/2.0);

        if (a == -1) {
          ROS_INFO("Arm returning home because planning failed");
          homeArm();
          failureReason = "planning";
          state = FAILURE;
          return;
        }
        if (!executeCurrentPlan()) {
          ROS_INFO("Arm returning home because execution failed");
          homeArm();
          failureReason = "execution";
          state = FAILURE;
          return;
        }

        ros::Duration(1.0).sleep();
        homeArm();
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
        tf::Transform rot = tf::Transform(yawPitchToQuat(yaw, *i));
        tf::Vector3 ob = tf::Vector3(objx, objy, objz);
        tf::Vector3 trans = rot*approachOffset;

        tf::Vector3 out = ob-trans;
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
        msg.failure_reason = failureReason;
        msg.obj_id = grabbedObject;
        statusPublisher.publish(msg);
        goalPublisher.publish(graspGoal);
     }

    void setUpScene()
    {
      std::vector<std::string> known = scene.getKnownObjectNames();
      scene.removeCollisionObjects(known);
      //ROS_INFO("Removing known collision objects");
      ros::Duration(1).sleep();

      boost::lock_guard<boost::mutex> guard(objMutex);
      std::vector<moveit_msgs::CollisionObject> coList;
      for (std::map<int, std::vector<float> >::iterator i = objectPoses.begin();
           i != objectPoses.end(); i++) {
          moveit_msgs::CollisionObject co;
          co.header.frame_id = group.getPlanningFrame();

          int objID = i->first;
          std::stringstream ss;
          ss << objID;
          co.id = ss.str();
          //ROS_INFO("Adding object %s", co.id.c_str());

          geometry_msgs::Pose box_pose;
          box_pose.position.x = i->second[0];
          box_pose.position.y = i->second[1];
          box_pose.position.z = i->second[2];

          geometry_msgs::Quaternion q;
          tf::quaternionTFToMsg(objectRotations[objID], q);
          box_pose.orientation = q;

          shape_msgs::SolidPrimitive primitive;
          primitive.type = primitive.BOX;
          primitive.dimensions.resize(3);
          primitive.dimensions[0] = objectSizes[objID][0]+0.02;
          primitive.dimensions[1] = objectSizes[objID][1]+0.02;
          primitive.dimensions[2] = objectSizes[objID][2]+0.02;

          co.primitives.push_back(primitive);
          co.primitive_poses.push_back(box_pose);
          co.operation = co.ADD;
          coList.push_back(co);
      }
      moveit_msgs::CollisionObject planeobj;
      planeobj.header.frame_id = group.getPlanningFrame();
      planeobj.id = "table";

      geometry_msgs::Pose planep;
      planep.position.x = 0.8;
      planep.position.y = 0.0;
      planep.position.z = ((currentTable[3] + currentTable[0]*0.8 +
                            currentTable[1]*0.0) / -currentTable[2]);
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

    void homeArm()
    {
        ros::Duration(0.1).sleep();

        std::vector<double> joints = std::vector<double>();
        joints.push_back(1.32);
        joints.push_back(0.7);
        joints.push_back(0.0);
        joints.push_back(-2.0);
        joints.push_back(0.0);
        joints.push_back(-0.57);
        joints.push_back(0.0);
        group.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroup::Plan homePlan;

        int tries = 0;
        std::string failure = "";
        moveit::planning_interface::MoveItErrorCode success;
        while(tries < numRetries) {
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

        if (!success) {
          ROS_INFO("Homing arm failed.");
          //failureReason = failure;
          //state = FAILURE;
          return;
        }
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

  geometry_msgs::Pose xyzypTargetToPoseMsg(float x, float y, float z,
                                           float yaw, float pitch)
  {
    tf::Quaternion q = yawPitchToQuat(yaw, pitch);
    geometry_msgs::Pose p = geometry_msgs::Pose();
    geometry_msgs::Quaternion tq;
    tf::quaternionTFToMsg(q, tq);
    p.orientation = tq;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;

    return p;
  }


  bool planToXYZQuaternionTarget(float x, float y, float z, tf::Quaternion q)
  {
    group.setStartStateToCurrentState();
    geometry_msgs::Pose target = geometry_msgs::Pose();
    geometry_msgs::Quaternion tq;
    tf::quaternionTFToMsg(q, tq);
    target.orientation = tq;

    std::vector<float> targ = fingertipToEEFrame(x, y, z, q);
    target.position.x = targ[0];
    target.position.y = targ[1];
    target.position.z = targ[2];

    geometry_msgs::Pose fingerTarget = geometry_msgs::Pose();
    fingerTarget.orientation = tq;
    fingerTarget.position.x = x;
    fingerTarget.position.y = y;
    fingerTarget.position.z = z;

    graspGoal.pose = fingerTarget;
    graspGoal.header.frame_id = group.getPlanningFrame();
    ros::Duration(1.0).sleep();

    group.setPoseTarget(target);
    moveit::planning_interface::MoveGroup::Plan xyzPlan;

    moveit::planning_interface::MoveItErrorCode success = group.plan(xyzPlan);
    // To stop it thinking it's successful if postprocessing is the problem
    if (xyzPlan.trajectory_.joint_trajectory.points.empty() &&
        xyzPlan.trajectory_.multi_dof_joint_trajectory.points.empty()) {
      success = false;
    }

    if (!success) {
      currentPlan = moveit::planning_interface::MoveGroup::Plan();
      return false;
    }

    currentPlan = xyzPlan;
    return true;
  }

    bool planToXYZAngleTarget(float x, float y, float z, float pitch, float yaw)
    {
      tf::Quaternion q = yawPitchToQuat(yaw, pitch);
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
    tf::Pose t;
    tf::poseMsgToTF(p, t);
    tf::Transform rotationX(t.getRotation());

    tf::Vector3 trans = rotationX*tf::Vector3(-fingerToWrist[0],
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
                                        tf::Quaternion q)
  {
    std::vector<float> finger;
    finger.push_back(fx);
    finger.push_back(fy);
    finger.push_back(fz);

    return fingertipToEEFrame(finger, q);
  }

  std::vector<float> fingertipToEEFrame(std::vector<float> fingertip,
                                        tf::Quaternion q)
  {
    tf::Transform rot = tf::Transform(q);
    tf::Vector3 ft = tf::Vector3(fingertip[0],
                                 fingertip[1],
                                 fingertip[2]);
    tf::Vector3 trans = rot*tf::Vector3(fingerToWrist[0],
                                        fingerToWrist[1],
                                        fingerToWrist[2]);

    tf::Vector3 out = ft+trans;

    std::vector<float> res;
    res.push_back(out.x());
    res.push_back(out.y());
    res.push_back(out.z());

    return res;
  }

  std::vector<float> fingertipToEEFrame(std::vector<float> fingertip,
                                        std::vector<float> handRPY)
  {
    tf::Quaternion q = tf::createQuaternionFromRPY(handRPY[0],
                                                   handRPY[1],
                                                   handRPY[2]);
    return fingertipToEEFrame(fingertip, q);
  }

private:
    ros::NodeHandle n;
    ros::Subscriber obsSubscriber;
    ros::Subscriber commSubscriber;
    ros::Subscriber jointsSubscriber;
    ros::Publisher statusPublisher;
    ros::Publisher goalPublisher;
    geometry_msgs::PoseStamped graspGoal;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
    ros::Timer pubTimer;

    ActionState state;
    moveit::planning_interface::MoveGroup::Plan currentPlan;
    std::string failureReason;
    int numRetries;
    long lastCommandTime;
    int grabbedObject;
    std::vector<float> grabbedObjSize;
    bool gripperClosed;
    bool checkPlans;
    bool isSimRobot;
    tf::Vector3 fingerToWrist;

    // Grasp position variations
    std::vector<float> approachAngles;
    float preferredDropAngle;
    tf::Vector3 approachOffset;
    tf::Vector3 grabMotion;
    tf::Vector3 dropMotion;
    float pushOffset;

    moveit::planning_interface::MoveGroup group;
    moveit::planning_interface::PlanningSceneInterface scene;

    std::map<int, std::vector<float> > objectPoses;
    std::map<int, tf::Quaternion> objectRotations;
    std::map<int, std::vector<float> > objectSizes;
    std::vector<float> currentTable;
    boost::mutex objMutex;
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
