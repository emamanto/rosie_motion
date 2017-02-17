#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rosie_msgs.msg import RobotCommand, RobotAction

def callback(data):
    rospy.loginfo("Received a message!")

# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    rospy.init_node("rosie_motion_server")
    rospy.loginfo("Rosie motor node starting up...")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    rospy.Subscriber("/rosie_arm_commands", RobotCommand, callback)

    while not rospy.is_shutdown():
        rospy.loginfo("Rosie motor node running!")
        time.sleep(5)
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
