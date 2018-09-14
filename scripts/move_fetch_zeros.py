#!/usr/bin/env python

# Sends motion command as if from Rosie
import rospy
import time
from gazebo_msgs.msg import ModelState

if __name__ == '__main__':
    rospy.init_node("move_gazebo_fetch_model")

    pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

    msg = ModelState()
    msg.model_name = "fetch"
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0

    for i in range(0, 10):
        pub.publish(msg)
        time.sleep(0.1)
