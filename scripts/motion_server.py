#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
import sys
import numpy
from threading import Lock
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from rosie_msgs.msg import RobotCommand, RobotAction, Observations

def command_callback(data, state):
    if data.action == state.action_state:
        return

    state.action_state = data.action
    if robot_state.GRAB in data.action:
        targ = data.action.split('=')[1]
        state.to_grab = int(targ)
    else:
        state.to_grab = -1

    rospy.loginfo("Going to make a motion plan!")
    pose_target = geometry_msgs.msg.Pose()
    plan_target = []
    state.obj_lock.acquire()
    try:
        plan_target = [state.perceived_objects[state.to_grab].translation.x,
                       state.perceived_objects[state.to_grab].translation.y,
                       state.perceived_objects[state.to_grab].translation.z,
                       1]
    finally:
        state.obj_lock.release()

    world2robot = [[1, 0, 0, 0.8],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.695],
                   [0, 0, 0, 1]]
    adjusted_target = numpy.dot(world2robot, plan_target)
    pose_target.orientation.w = 1.0
    pose_target.position.x = adjusted_target[0]
    pose_target.position.y = adjusted_target[1]
    pose_target.position.z = adjusted_target[2] + 0.2

    print(adjusted_target)

    state.group.set_pose_target(pose_target)
    state.group.plan()
    state.group.go(wait=True)

def object_callback(data, state):
    state.obj_lock.acquire()
    state.perceived_objects.clear()
    try:
        for o in data.observations:
            state.perceived_objects[o.obj_id] = o.bbox_xyzrpy
    finally:
        state.obj_lock.release()

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

        self.obj_lock = Lock()
        self.perceived_objects = {}

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("rosie_motion_server")
        rospy.loginfo("Rosie motor node starting up...")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")

        rospy.Subscriber("/rosie_arm_commands", RobotCommand, command_callback, callback_args=self)
        rospy.Subscriber("/rosie_observations", Observations, object_callback, callback_args=self)

        self.stats_pub = rospy.Publisher("/rosie_arm_status", RobotAction, queue_size=10)
        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            #rospy.loginfo("ROSIE!")
            msg = RobotAction()
            msg.utime = long(time.time()*1000)
            msg.action = self.action_state
            msg.obj_id = 0
            self.stats_pub.publish(msg)
            self.rate.sleep()


# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    time.sleep(15)
    status = robot_state()
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    group.get_move_action().cancel_all_goals()
