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
    enum ActionState {WAIT,
                      HOME,
                      GRAB,
                      POINT,
                      DROP,
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
            case FAILURE: return "FAILURE";
            case SCENE: return "SCENE";
            default: return "WTF";
        }
    }

  MotionServer(bool humanCheck=true) : state(WAIT),
                                       failureReason("none"),
                                       lastCommandTime(0),
                                       grabbedObject(-1),
                                       checkPlans(humanCheck),
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

        group.setMaxVelocityScalingFactor(0.3);

        obsSubscriber = n.subscribe("rosie_observations", 10,
                                    &MotionServer::obsCallback, this);
        commSubscriber = n.subscribe("rosie_arm_commands", 10,
                                     &MotionServer::commandCallback, this);
        statusPublisher = n.advertise<rosie_msgs::RobotAction>("rosie_arm_status", 10);
        goalPublisher = n.advertise<geometry_msgs::PoseStamped>("rosie_grasp_target", 10);

        gripper.waitForServer();
        closeGripper();

        graspGoal.pose.position.x = 0;
        graspGoal.pose.position.y = 0;
        graspGoal.pose.position.z = 0;
        graspGoal.pose.orientation.w = 1.0;
        graspGoal.header.frame_id = group.getPlanningFrame();

        pubTimer = n.createTimer(ros::Duration(0.1),
                                 &MotionServer::publishStatus, this);

        ROS_INFO("RosieMotionServer READY!");
    };

    void obsCallback(const rosie_msgs::Observations::ConstPtr& msg)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);

        objectPoses.clear();
        objectSizes.clear();

        currentTable = msg->table;

        for (std::vector<rosie_msgs::ObjectData>::const_iterator i = msg->observations.begin();
             i != msg->observations.end(); i++) {
            std::vector<float> pos = std::vector<float>();
            pos.push_back(i->bbox_xyzrpy.translation.x);
            pos.push_back(i->bbox_xyzrpy.translation.y);
            pos.push_back(i->bbox_xyzrpy.translation.z);
            tf::Quaternion quat;
            tf::quaternionMsgToTF(i->bbox_xyzrpy.rotation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            pos.push_back(float(roll));
            pos.push_back(float(pitch));
            pos.push_back(float(yaw));
            objectPoses.insert(std::pair<int, std::vector<float> >(i->obj_id, pos));

            std::vector<float> dim = std::vector<float>();
            dim.push_back(i->bbox_dim.x);
            dim.push_back(i->bbox_dim.y);
            dim.push_back(i->bbox_dim.z);
            objectSizes.insert(std::pair<int, std::vector<float> >(i->obj_id, dim));
        }
    }

    void commandCallback(const rosie_msgs::RobotCommand::ConstPtr& msg)
    {
      if (asToString(state) == msg->action || msg->utime == lastCommandTime)
        return;

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
            failureReason = "none";
            state = WAIT;
        }
        else if (msg->action.find("RESET")!=std::string::npos){
            ROS_INFO("Handling reset command");
            state = HOME;
            homeArm();
            failureReason = "none";
            state = WAIT;
        }
        else if (msg->action.find("SCENE")!=std::string::npos){
            ROS_INFO("Handling build scene command");
            state = SCENE;
            setUpScene();
            failureReason = "none";
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
          failureReason = "invalidpickup";
          state = FAILURE;
          return;
        }

        float x = objectPoses[id][0] - (objectSizes[id][0]/2.0) - 0.16;
        float y = objectPoses[id][1];
        float z = objectPoses[id][2] + (objectSizes[id][2]/2.0) + 0.22;

        if (isSimRobot) y -= 0.02;

        bool reachSuccess = moveToXYZTarget(x, y, z);
        if (!reachSuccess) return;

        ros::Duration(0.5).sleep();
        openGripper();

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(group.getCurrentPose().pose);
        geometry_msgs::Pose gp = waypoints[0];
        gp.position.x += 0.06;
        gp.position.z -= 0.08;
        waypoints.push_back(gp);

        group.setStartStateToCurrentState();
        moveit_msgs::RobotTrajectory inTraj;
        double frac = group.computeCartesianPath(waypoints,
                                                 0.01, 0.0,
                                                 inTraj,
                                                 false);
        if (!safetyCheck()) {
          failureReason = "safety";
          state = FAILURE;
          return;
        }

        moveit::planning_interface::MoveGroup::Plan p;
        p.trajectory_ = inTraj;
        moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(p);
        if (!moveSuccess) {
          ROS_INFO("Execution failed with error code %d", moveSuccess.val);
          failureReason = "execution";
        }

        closeGripper();
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
        geometry_msgs::Pose bp = waypoints2[0];
        bp.position.x -= 0.08;
        bp.position.z += 0.08;
        waypoints2.push_back(bp);

        group.setStartStateToCurrentState();
        moveit_msgs::RobotTrajectory outTraj;
        double frac2 = group.computeCartesianPath(waypoints2,
                                                 0.01, 0.0,
                                                 outTraj,
                                                 false);
        if (!safetyCheck()) {
          failureReason = "safety";
          state = FAILURE;
          return;
        }

        moveit::planning_interface::MoveGroup::Plan p2;
        p2.trajectory_ = outTraj;
        moveit::planning_interface::MoveItErrorCode outSuccess = group.execute(p2);
        if (!outSuccess) ROS_INFO("Execution failed with error code %d",
                                   outSuccess.val);

        homeArm();
        failureReason = "none";
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
      if (isSimRobot) target[1] -= 0.02;

      bool reachSuccess = moveToXYZTarget(target[0] - (grabbedObjSize[0]/2.0) - 0.15,
                                          target[1],
                                          target[2] + (grabbedObjSize[2]/2.0) + 0.2);

      if (!reachSuccess) return;

      ros::Duration(0.5).sleep();
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(group.getCurrentPose().pose);
      geometry_msgs::Pose gp = waypoints[0];
      gp.position.x += 0.05;
      gp.position.z -= 0.03;
      waypoints.push_back(gp);

      group.setStartStateToCurrentState();
      moveit_msgs::RobotTrajectory inTraj;
      double frac = group.computeCartesianPath(waypoints,
                                               0.01, 0.0,
                                               inTraj,
                                               false);
      if (!safetyCheck()) {
        failureReason = "safety";
        state = FAILURE;
        return;
      }

      moveit::planning_interface::MoveGroup::Plan p;
      p.trajectory_ = inTraj;
      moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(p);
      if (!moveSuccess) ROS_INFO("Execution failed with error code %d",
                                 moveSuccess.val);

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
      dropP.position.z = target[2] + grabbedObjSize[2] + 0.02;
      dropP.orientation.w = 1.0;

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = grabbedObjSize[0]+0.02;
      primitive.dimensions[1] = grabbedObjSize[1]+0.02;
      primitive.dimensions[2] = grabbedObjSize[2]+0.02;

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
      geometry_msgs::Pose bp = waypoints2[0];
      bp.position.x -= 0.08;
      bp.position.z += 0.08;
      waypoints2.push_back(bp);

      group.setStartStateToCurrentState();
      moveit_msgs::RobotTrajectory outTraj;
      double frac2 = group.computeCartesianPath(waypoints2,
                                                0.01, 0.0,
                                                outTraj,
                                                false);
      if (!safetyCheck()) {
        failureReason = "safety";
        state = FAILURE;
        return;
      }

      moveit::planning_interface::MoveGroup::Plan p2;
      p2.trajectory_ = outTraj;
      moveit::planning_interface::MoveItErrorCode outSuccess = group.execute(p2);
      if (!outSuccess) ROS_INFO("Execution failed with error code %d",
                                outSuccess.val);


      ros::Duration(0.5).sleep();

      closeGripper();
      homeArm();
    }

    void handlePointCommand(int id)
    {
        boost::lock_guard<boost::mutex> guard(objMutex);
        if (objectSizes.find(id) == objectSizes.end()||
            objectPoses.find(id) == objectSizes.end()) {
          ROS_INFO("Object ID %d is not being perceived", id);
          return;
        }

        float x = objectPoses[id][0] - (objectSizes[id][0]/2.0) - 0.2;
        float y = objectPoses[id][1];
        float z = objectPoses[id][2] + (objectSizes[id][2]/2.0) + 0.2;

        if (!moveToXYZTarget(x, y, z)) return;
        ros::Duration(1.0).sleep();
        homeArm();
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

          geometry_msgs::Pose box_pose;
          box_pose.position.x = i->second[0];
          box_pose.position.y = i->second[1];
          box_pose.position.z = i->second[2] + objectSizes[objID][2]/2.0;
          if (isSimRobot) {
              box_pose.position.y -= 0.02;
              box_pose.position.x += 0.01;
          }
          box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(i->second[5],
                                                                         i->second[4],
                                                                         -i->second[3]);

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
          //ROS_INFO("Adding %d", objID);
      }
      moveit_msgs::CollisionObject planeobj;
      planeobj.header.frame_id = group.getPlanningFrame();
      planeobj.id = "table";

      geometry_msgs::Pose planep;
      planep.position.x = 0.8;
      planep.position.y = 0.0;
      planep.position.z = ((currentTable[3] + currentTable[0]*0.8 +
                            currentTable[1]*0.0) / -currentTable[2]) + 0.02;
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
        group.setStartStateToCurrentState();
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
        moveit::planning_interface::MoveItErrorCode success = group.plan(homePlan);
        if (!success) {
          ROS_INFO("Planning failed with error code %d", success.val);
          failureReason = "planning";
          state = FAILURE;
          return;
        }

        if (!safetyCheck()) {
          failureReason = "safety";
          state = FAILURE;
          return;
        }

        moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(homePlan);
        if (!moveSuccess) {
          ROS_INFO("Execution failed with error code %d", moveSuccess.val);
          failureReason = "execution";
          state = FAILURE;
        } else {
          failureReason = "none";
          state = WAIT;
        }
    }

  void closeGripper()
  {
    control_msgs::GripperCommandGoal gripperGoal;
    gripperGoal.command.max_effort = 0.0;
    gripperGoal.command.position = 0.0;

    gripper.sendGoal(gripperGoal);
    gripper.waitForResult(ros::Duration(2.0));
  }

  void openGripper()
  {
    control_msgs::GripperCommandGoal gripperGoal;
    gripperGoal.command.max_effort = 0.0;
    gripperGoal.command.position = 0.1;

    gripper.sendGoal(gripperGoal);
    gripper.waitForResult(ros::Duration(2.0));
  }

    bool moveToXYZTarget(float x, float y, float z)
    {
      group.setStartStateToCurrentState();
        geometry_msgs::Quaternion q =
            tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/4.0, 0);
        geometry_msgs::Pose target = geometry_msgs::Pose();
        target.orientation = q;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        graspGoal.pose = target;
        graspGoal.header.frame_id = group.getPlanningFrame();
        ros::Duration(1.0).sleep();

        group.setPoseTarget(target);
        moveit::planning_interface::MoveGroup::Plan xyzPlan;
        moveit::planning_interface::MoveItErrorCode success = group.plan(xyzPlan);
        if (!success) {
          ROS_INFO("Planning failed with error code %d", success.val);
          failureReason = "planning";
          state = FAILURE;
          return false;
        }

        if (!safetyCheck()) {
          failureReason = "safety";
          state = FAILURE;
          return false;
        }

        moveit::planning_interface::MoveItErrorCode moveSuccess = group.execute(xyzPlan);
        if (!moveSuccess) {
          ROS_INFO("Execution failed with error code %d", moveSuccess.val);
          failureReason = "execution";
          state = FAILURE;
          return false;
        }

        return true;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber obsSubscriber;
    ros::Subscriber commSubscriber;
    ros::Publisher statusPublisher;
    ros::Publisher goalPublisher;
    geometry_msgs::PoseStamped graspGoal;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
    ros::Timer pubTimer;

    ActionState state;
    std::string failureReason;
    long lastCommandTime;
    int grabbedObject;
    std::vector<float> grabbedObjSize;
    bool checkPlans;
    bool isSimRobot;

    moveit::planning_interface::MoveGroup group;
    moveit::planning_interface::PlanningSceneInterface scene;

    std::map<int, std::vector<float> > objectPoses;
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
