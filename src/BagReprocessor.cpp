#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "ArmController.h"

class BagReprocessor {
public:
    BagReprocessor() : ready(false),
                       arm(n, true) {
        std::string bagName;
        if (!n.getParam("/rosie_bag_reprocessor/filename", bagName)) {
            ROS_WARN("BagReprocessor is missing a filename!");
            return;
        }

        try {
            bagFile.open(bagName, rosbag::bagmode::Read);
        } catch (rosbag::BagException e) {
            ROS_WARN("BagReprocessor error: %s", e.what());
            return;
        }

        arm.setHumanChecks(false);
        arm.setLibrary("ompl");
        arm.setPlanner("rrtc");
        arm.setPlanningTime(0);

        ros::Duration(30.0).sleep();

        ready = true;
    }

    void writeInfo(std_msgs::String::ConstPtr pl_,
                   std_msgs::Float32::ConstPtr pt_,
                   sensor_msgs::JointState::ConstPtr ss_,
                   geometry_msgs::Transform::ConstPtr xf_,
                   trajectory_msgs::JointTrajectory::ConstPtr traj_)
{
        arm.setPlanner(pl_->data);

        moveit_msgs::RobotTrajectory rt;
        rt.joint_trajectory = *traj_;

        tf2::Transform tf2Xf;
        tf2::fromMsg(*xf_, tf2Xf);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = rt;
        plan.planning_time_ = pt_->data;
        plan.start_state_.joint_state = *ss_;
        plan.start_state_.is_diff = false;

        arm.writeQuery(tf2Xf, plan);
}

    bool process() {
        std::vector<std::string> scTopic;
        scTopic.push_back(std::string("scenes"));

        rosbag::View scView(bagFile, rosbag::TopicQuery(scTopic));

        int numScenes = 0;
        moveit_msgs::PlanningSceneWorld::ConstPtr s;

        for(rosbag::MessageInstance const m: scView)
        {
            if (numScenes > 0) {
                ROS_WARN("Multiple planning scenes found in experiment log!");
                return false;
            }

            s = m.instantiate<moveit_msgs::PlanningSceneWorld>();
            numScenes++;

            if (s == NULL) {
                ROS_WARN("Planning scene message error!");
                return false;
            }
        }
        if (numScenes == 0) {
            ROS_WARN("Zero planning scenes found in experiment log!");
            return false;
        }

        arm.updateCollisionScene(s->collision_objects);


        std::vector<std::string> dataTopics;
        dataTopics.push_back(std::string("planners"));
        dataTopics.push_back(std::string("targets"));
        dataTopics.push_back(std::string("planning_times"));
        dataTopics.push_back(std::string("start_states"));
        dataTopics.push_back(std::string("trajectories"));

        rosbag::View dataView(bagFile, rosbag::TopicQuery(dataTopics));

        std_msgs::String::ConstPtr pl;
        std_msgs::Float32::ConstPtr pt;
        sensor_msgs::JointState::ConstPtr ss;
        geometry_msgs::Transform::ConstPtr xf;
        trajectory_msgs::JointTrajectory::ConstPtr traj;

        // pl, pt, ss, xf, traj
        std::vector<ros::Time> msgTimes;
        msgTimes.resize(5);
        msgTimes[0] = ros::Time::now();

        int numTrajectories = 0;

        for (rosbag::MessageInstance const m: dataView)
        {
            bool allMatched = true;
            for (int i = 1; i < msgTimes.size(); i++) {
                if (msgTimes[i] != msgTimes[i-1]) {
                    allMatched = false;
                    break;
                }
            }

            if (allMatched) {
                numTrajectories++;
                writeInfo(pl, pt, ss, xf, traj);
            }

            std_msgs::String::ConstPtr tmpPl = m.instantiate<std_msgs::String>();
            if (tmpPl != NULL) {
                pl = tmpPl;
                msgTimes[0] = m.getTime();
                continue;
            }

            std_msgs::Float32::ConstPtr tmpPt = m.instantiate<std_msgs::Float32>();
            if (tmpPt != NULL) {
                pt = tmpPt;
                msgTimes[1] = m.getTime();
                continue;
            }

            sensor_msgs::JointState::ConstPtr tmpSs = m.instantiate<sensor_msgs::JointState>();
            if (tmpSs != NULL) {
                ss = tmpSs;
                msgTimes[2] = m.getTime();
                continue;
            }

            geometry_msgs::Transform::ConstPtr tmpXf = m.instantiate<geometry_msgs::Transform>();
            if (tmpXf != NULL) {
                xf = tmpXf;
                msgTimes[3] = m.getTime();
                continue;
            }

            trajectory_msgs::JointTrajectory::ConstPtr tmpTraj = m.instantiate<trajectory_msgs::JointTrajectory>();
            if (tmpTraj != NULL) {
                traj = tmpTraj;
                msgTimes[4] = m.getTime();
                continue;
            }
        }
        // Check for last set of msgs
        bool allMatched = true;
        for (int i = 1; i < msgTimes.size(); i++) {
            if (msgTimes[i] != msgTimes[i-1]) {
                allMatched = false;
                break;
            }
        }

        if (allMatched) {
            numTrajectories++;
            writeInfo(pl, pt, ss, xf, traj);
        } else {
            ROS_WARN("Final set of messages incomplete!");
        }

        ROS_INFO("Found %i trajectories in the .bag file!", numTrajectories);
        return true;
    }

    bool fileReady() {
        return ready;
    }

private:
    ros::NodeHandle n;
    rosbag::Bag bagFile;
    ArmController arm;

    bool ready;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_bag_reprocessor");
    BagReprocessor br;

    ros::AsyncSpinner mainSpin(4);
    mainSpin.start();

    if (br.fileReady() && br.process()) {
        ROS_INFO("BagReprocessor finished with the file successfully!");
    } else {
        ROS_WARN("BagReprocessor encountered an error!");
    }

    return 0;
}
