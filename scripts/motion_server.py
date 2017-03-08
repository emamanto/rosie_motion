#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
import sys
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from rosie_msgs.msg import RobotCommand, RobotAction, Observations

class robot_state:
    WAIT = "WAIT"
    HOME = "HOME"
    GRAB = "GRAB"
    POINT = "POINT"
    DROP = "DROP"
    FAILURE = "FAILURE"

    def __init__(self):
        self.action_state = robot_state.WAIT
        self.grabbed_object = -1
        self.to_grab = -1

def command_callback(data):
    rospy.loginfo("Received a command message!")

def object_callback(data):
    rospy.loginfo("Received an object message!")

# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    status = robot_state()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("rosie_motion_server")
    rospy.loginfo("Rosie motor node starting up...")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    rospy.loginfo("-------PLANNING FRAME--------- " + group.get_planning_frame())

    rospy.Subscriber("/rosie_arm_commands", RobotCommand, command_callback)
    rospy.Subscriber("/rosie_observations", Observations, object_callback)

    stats_pub = rospy.Publisher("/rosie_arm_status", RobotAction, queue_size=10)
    rate = rospy.Rate(1)

    time.sleep(30)

    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.w = 1.0
    # pose_target.position.x = 0.7
    # pose_target.position.y = -0.05
    # pose_target.position.z = 1.1
    # group.set_pose_target(pose_target)
    # group.plan()
    # group.go(wait=True)

    msg = RobotAction()
    msg.utime = long(time.time()*1000)
    msg.action = "wait"
    msg.obj_id = 0

    while not rospy.is_shutdown():
        stats_pub.publish(msg)
        rate.sleep()
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    group.get_move_action().cancel_all_goals()
