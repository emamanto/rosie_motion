#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "ObjectDatabase.h"
#include "WorldObjects.h"
#include "ArmController.h"

class BagReprocessor {
public:
    BagReprocessor() : ready(false),
                       arm(n) {
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
        return true;
    }

    bool fileReady() {
        return ready;
    }

private:
    ros::NodeHandle n;
    rosbag::Bag bagFile;
    //WorldObjects world;
    //ObjectDatabase objData;
    ArmController arm;

    bool ready;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_bag_reprocessor");
    BagReprocessor br;

    if (br.fileReady() && br.process()) {
        ROS_INFO("BagReprocessor finished with the file successfully!");
    } else {
        ROS_WARN("BagReprocessor encountered an error!");
    }

    return 0;
}
