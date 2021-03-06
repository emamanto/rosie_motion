#!/usr/bin/env python

# Sends motion command as if from Rosie
import rospy
import time
from rosie_msgs.msg import RobotCommand

if __name__ == '__main__':
    rospy.init_node("rosie_point2")

    objname = raw_input("Object name? ")

    pub = rospy.Publisher("/rosie_arm_commands", RobotCommand, queue_size=10)

    msg = RobotCommand()
    msg.action = "POINT=" + objname
    msg.utime = long(time.time()*1000)

    for i in range(0, 10):
        pub.publish(msg)
        time.sleep(0.1)
