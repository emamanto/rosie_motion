#!/usr/bin/env python

# Sends motion command as if from Rosie
import rospy
import time
from rosie_msgs.msg import RobotCommand
from geometry_msgs.msg import Transform

if __name__ == '__main__':
    rospy.init_node("rosie_pickup")

    targ = input("Location? (x for random) ")
    nums = ["0.7", "0.1", "0"]
    if targ != "x":
        nums = targ.split(" ")

    dest = Transform()
    dest.translation.x = float(nums[0])
    dest.translation.y = float(nums[1])
    dest.translation.z = 0
    dest.rotation.w = 1.0

    pub = rospy.Publisher("/rosie_arm_commands", RobotCommand, queue_size=10)

    msg = RobotCommand()
    msg.action = "DROP"
    msg.utime = long(time.time()*1000)
    msg.updateDest = False
    msg.dest = dest

    for i in range(0, 10):
        pub.publish(msg)
        time.sleep(0.1)
