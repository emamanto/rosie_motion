#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <ros/ros.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <actionlib/client/simple_action_client.h>

#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "moveit_msgs/ApplyPlanningScene.h"
#include "moveit_msgs/GetPlanningScene.h"
#include "moveit_msgs/GetMotionPlan.h"

class ArmController {
public:
    typedef std::vector<moveit::planning_interface::MoveGroupInterface::Plan> PlanVector;
    typedef std::pair<tf2::Transform, tf2::Transform> GraspPair;

    // Independent of the specific setup
    static double jointLength(moveit_msgs::RobotTrajectory traj);
    static double execTime(moveit_msgs::RobotTrajectory traj);

    // Dependent on the specific setup
    double handLength(moveit_msgs::RobotTrajectory traj);
    // Returns min clearance, avg clearance metric
    std::vector<double> clearanceData(moveit_msgs::RobotTrajectory traj);

    ArmController(ros::NodeHandle& nh);
    void setHumanChecks(bool on) { checkPlans = on; }
    void setStomp(bool isStomp) { stomp = isStomp; }
    void setPlannerName(std::string n);
    void setPlanningTime(double t);

    std::string armPlanningFrame();
    std::string getHeld() { return grabbedObject.id; }

    void updateCollisionScene(std::vector<moveit_msgs::CollisionObject> cos);
    void attachToGripper(std::string objName);
    void detachHeldObject();

    void closeGripper();
    void openGripper();
    void setGripperClosed(bool isClosed);

    bool pickUp(tf2::Transform objXform,
                std::vector<GraspPair> graspList,
                std::string objName);
    bool putDownHeldObj(std::vector<tf2::Transform> targets);
    bool pointTo(tf2::Transform objXform,
                 float objHeight);
    bool planToTargetList(std::vector<tf2::Transform> targets, int numTrials);
    bool planToRegion(float xD, float yD, float zD, geometry_msgs::Pose p);
    bool planToRegionAsList(float xD, float yD, float zD, geometry_msgs::Pose p, int numTrials);
    bool checkIKPose(tf2::Transform blockXform);
    bool homeArm();

private:
    void setGripperTo(float m);
    bool planToXformInner(tf2::Transform t);
    bool planToXform(tf2::Transform t, int n);
    double planStraightLineMotion(tf2::Transform target);
    bool executeCurrentPlan();
    void writeQuery(tf2::Transform t,
                    moveit::planning_interface::MoveGroupInterface::Plan p);
    void writeHomeQuery(moveit::planning_interface::MoveGroupInterface::Plan p);
    void writeTrajectoryInfo(std::ofstream& ofs, moveit_msgs::RobotTrajectory& traj);
    bool safetyCheck();
    void publishCurrentGoal(const ros::TimerEvent& e);
    void setCurrentGoalTo(tf2::Transform t);

    std::string logFileName;

    moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
    int numRetries;
    moveit_msgs::CollisionObject grabbedObject;
    GraspPair usedGrasp;
    tf2::Transform prevObjRotation;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper;
    bool checkPlans;
    bool gripperClosed;
    bool stomp;
    std::string plannerName;
    double planningTime;

    geometry_msgs::PoseStamped currentGoal;
    ros::Publisher goalPublisher;
    ros::Timer pubTimer;

    moveit::planning_interface::MoveGroupInterface group;
    ros::ServiceClient psDiffClient;
    ros::ServiceClient getPSClient;
    planning_scene_monitor::PlanningSceneMonitorPtr psm;
    ros::ServiceClient planRequestClient;
};
