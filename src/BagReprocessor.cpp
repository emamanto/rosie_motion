#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
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
            ROS_INFO("BagReprocessor is missing a filename!");
            return;
        }

        try {
            bagFile.open(bagName, rosbag::bagmode::Read);
        } catch (rosbag::BagException e) {
            ROS_INFO("BagReprocessor error: %s", e.what());
            return;
        }

        arm.setHumanChecks(false);
        arm.setLibrary("ompl");
        arm.setPlanner("rrtc");
        arm.setPlanningTime(0);

        ready = true;
    }

    bool process() {
        return false;
    }

private:
    ros::NodeHandle n;
    rosbag::Bag bagFile;
    WorldObjects world;
    ObjectDatabase objData;
    ArmController arm;

    bool ready;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosie_bag_reprocessor");
    BagReprocessor br;

    if (br.process()) {
        ROS_INFO("BagReprocessor finished with the file successfully!");
    } else {
        ROS_WARN("BagReprocessor encountered an error!");
    }

    return 0;
}
