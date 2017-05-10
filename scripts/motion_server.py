#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
import sys
import copy
import math
import numpy
from threading import Lock, Thread
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from rosie_msgs.msg import RobotCommand, RobotAction, Observations
from quaternion import quat2rpy, rpy2quat

# Callback for when we get a command message
def command_callback(data, state):
    if state.action_state in data.action or data.utime == state.last_command_time:
        return

    state.action_state = data.action
    state.last_command_time = data.utime
    if robot_state.GRAB in data.action:
        targ = data.action.split('=')[1]
        state.to_grab = int(targ)
        handle_grasp(int(targ), state)
    if robot_state.POINT in data.action:
        targ = data.action.split('=')[1]
        handle_point(int(targ), state)
    else:
        state.to_grab = -1

def handle_grasp(id, state):
    rospy.loginfo("PICKING UP OBJECT WITH ID " + str(id))
    state.publish_status()
    plan_target = []
    state.obj_lock.acquire()
    try:
        plan_target = [state.perceived_objects[id].translation.x,
                       state.perceived_objects[id].translation.y,
                       state.perceived_objects[id].translation.z]
    finally:
        state.obj_lock.release()

    # Move to pre-grasp position
    state.move_to_xyz_target(plan_target)

    # Move in to grasp
    waypoints = []
    waypoints.append(state.group.get_current_pose().pose)

    grasp_pose = copy.deepcopy(waypoints[0])
    grasp_pose.position.x += 0.1
    grasp_pose.position.z -= 0.15
    waypoints.append(grasp_pose)

    (grasp_in, frac) = state.group.compute_cartesian_path(waypoints, 0.01, 0.0)
    state.group.execute(grasp_in)

    rospy.loginfo("Motor node switching to wait.")
    state.finished_action = robot_state.GRAB
    state.action_state = robot_state.WAIT

def handle_point(id, state):
    rospy.loginfo("POINTING TO OBJECT WITH ID " + str(id))
    state.publish_status()
    plan_target = []
    state.obj_lock.acquire()
    try:
        plan_target = [state.perceived_objects[id].translation.x,
                       state.perceived_objects[id].translation.y,
                       state.perceived_objects[id].translation.z]
    finally:
        state.obj_lock.release()

    # Move to pointing position
    state.move_to_xyz_target(plan_target)

    # Wait
    time.sleep(2)

    # Bring arm home so that we can see again
    state.home_arm()

    rospy.loginfo("Motor node switching to wait.")
    state.finished_action = robot_state.POINT
    state.action_state = robot_state.WAIT

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

    def publish_status(self):
            msg = RobotAction()
            msg.utime = long(time.time()*1000)
            msg.action = self.action_state
            msg.obj_id = self.grabbed_object
            self.stats_pub.publish(msg)

    def home_arm(self):
        joints = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        self.group.set_joint_value_target(joints)
        self.group.plan()
        self.group.go(wait=True)

    def move_to_xyz_target(self, target):
        world2robot = [[1, 0, 0, 0.8],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0.695],
                       [0, 0, 0, 1]]
        pt = [target[0], target[1], target[2], 1]
        adjusted_target = numpy.dot(world2robot, pt)
        target_rpy = [0, -math.pi/4.0, 0]
        target_quat = rpy2quat(target_rpy)

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = target_quat[0]
        pose_target.orientation.y = target_quat[1]
        pose_target.orientation.z = target_quat[2]
        pose_target.orientation.w = target_quat[3]
        pose_target.position.x = adjusted_target[0] - 0.2
        pose_target.position.y = adjusted_target[1]
        pose_target.position.z = adjusted_target[2] + 0.3

        self.group.set_pose_target(pose_target)
        self.group.plan()
        self.group.go(wait=True)

    def __init__(self):
        self.action_state = robot_state.WAIT
        self.last_command_time = 0
        self.grabbed_object = 0

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
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.loginfo("Rosie spinning")
            self.publish_status()
            self.rate.sleep()


# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    time.sleep(15)
    status = robot_state()
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    group.get_move_action().cancel_all_goals()
