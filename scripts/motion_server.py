#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
import sys
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from rosie_msgs.msg import RobotCommand, RobotAction

def callback(data):
    rospy.loginfo("Received a message!")

# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("rosie_motion_server")
    rospy.loginfo("Rosie motor node starting up...")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    rospy.Subscriber("/rosie_arm_commands", RobotCommand, callback)

    time.sleep(30)

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.7
    pose_target.position.y = -0.05
    pose_target.position.z = 1.1
    group.set_pose_target(pose_target)
    group.plan()
    group.go(wait=True)

    while not rospy.is_shutdown():
        rospy.loginfo("Rosie motor node running!")
        time.sleep(5)
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    group.get_move_action().cancel_all_goals()
