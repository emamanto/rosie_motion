/**
 *
 * Motion server v2.0
 *
 **/

#include <string>
#include <map>
#include <iostream>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_datatypes.h>

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
                                         lastCommandTime(0),
                                         grabbedObject(-1),
                                         checkPlans(humanCheck)
    {
        ROS_INFO("RosieMotionServer starting up!");
        ros::NodeHandle n;

        moveit::planning_interface::MoveGroup group("arm");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
        ROS_INFO("EE frame: %s", group.getEndEffectorLink().c_str());

        obsSubscriber = n.subscribe("rosie_observations", 10,
                                    &MotionServer::obsCallback, this);
        statusPublisher = n.advertise<rosie_msgs::RobotAction>("rosie_arm_status", 10);
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
            pos.push_back(i->bbox_dim.x);
            pos.push_back(i->bbox_dim.y);
            pos.push_back(i->bbox_dim.z);
            objectSizes.insert(std::pair<int, std::vector<float> >(i->obj_id, dim));
        }
    }

    void publish_status()
    {
        rosie_msgs::RobotAction msg = rosie_msgs::RobotAction();
        msg.utime = ros::Time::now().toNSec();
        msg.action = asToString(state).c_str();
        msg.obj_id = grabbedObject;
        statusPublisher.publish(msg);
     }

    void runLoop()
    {
        ROS_INFO("Node is running");
        publish_status();
    }

private:
    ros::Subscriber obsSubscriber;
    ros::Publisher statusPublisher;

    ActionState state;
    long lastCommandTime;
    int grabbedObject;
    bool checkPlans;

    std::map<int, std::vector<float> > objectPoses;
    std::map<int, std::vector<float> > objectSizes;
    std::vector<float> currentTable;
    boost::mutex objMutex;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_motion_server");
    MotionServer ms;

    ros::Rate loopRate(1);
    while (ros::ok()) {
        ros::spinOnce();
        ms.runLoop();
        loopRate.sleep();
    }

    return 0;
}
