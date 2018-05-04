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
                   state(WAIT),
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

    if (msg->position[pos1] < 0.009 && msg->position[pos2] < 0.009) {
      arm.setGripperClosed(true);
    } else {
      arm.setGripperClosed(false);
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
        ROS_INFO("Handling GRAB command for %s", name.c_str());
        handleGrabCommand(name);
    }
    else if (msg->action.find("DROP")!=std::string::npos) {
      ROS_INFO("Handling DROP command");
      state = DROP;
      std::vector<float> t = std::vector<float>();
      t.push_back(msg->dest.translation.x);
      t.push_back(msg->dest.translation.y);
      t.push_back(msg->dest.translation.z);
      handleDropCommand(t);
    }
    else if (msg->action.find("POINT")!=std::string::npos) {
      ROS_INFO("Handling POINT command");
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
    else if (msg->action.find("SCENE")!=std::string::npos){
      ROS_INFO("Handling build scene command");
      state = SCENE;
      arm.updateCollisionScene(getCollisionModels());
      state = WAIT;
    }
    else if (msg->action.find("RELOAD")!=std::string::npos){
      ROS_INFO("Handling reload object database command");
      objData.reload();
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

    if (success) {
      state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    } else {
      state = FAILURE;
      ROS_INFO("Arm status is now FAILURE");
    }
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

    if (success) {
      state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    } else {
      state = FAILURE;
      ROS_INFO("Arm status is now FAILURE");
    }
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

    arm.updateCollisionScene(getCollisionModels());
    bool success = arm.pointTo(world.getXformOf(objID), shapeHeight);
    if (success) {
      state = WAIT;
      ROS_INFO("Arm status is now WAIT");
    } else {
      state = FAILURE;
      ROS_INFO("Arm status is now FAILURE");
    }
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
