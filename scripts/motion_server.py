#!/usr/bin/env python

# Motor control command interpreter for rosie
import time
import rospy
import sys
import copy
import math
import numpy
from threading import Lock
import actionlib
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from rosie_msgs.msg import RobotCommand, RobotAction, Observations
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
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
    elif robot_state.POINT in data.action:
        targ = data.action.split('=')[1]
        state.to_grab = int(targ)
        handle_point(int(targ), state)
    elif robot_state.HOME in data.action:
        state.to_grab = -1
        handle_home(state)
    else:
        state.to_grab = -1

def handle_home(state):
    state.home_arm()

def handle_grasp(id, state):
    rospy.loginfo("PICKING UP OBJECT WITH ID " + str(id))
    state.publish_status()
    plan_target = []
    state.obj_lock.acquire()
    try:
        plan_target = [state.perceived_objects[id].translation.x-0.22,
                       state.perceived_objects[id].translation.y-0.02,
                       state.perceived_objects[id].translation.z+0.22]
    finally:
        state.obj_lock.release()

    # Move to pre-grasp position
    state.move_to_xyz_target(plan_target)
    rospy.sleep(2)

    state.open_gripper()

    # Move in to grasp
    waypoints = []
    waypoints.append(state.group.get_current_pose().pose)

    grasp_pose = copy.deepcopy(waypoints[0])
    grasp_pose.position.x += 0.08
    grasp_pose.position.z -= 0.08
    waypoints.append(grasp_pose)

    (grasp_in, frac) = state.group.compute_cartesian_path(waypoints,
                                                          0.01, 0.0,
                                                          avoid_collisions=False)

    if state.check_motion:
        goahead = raw_input("Is this plan okay? ")
        if goahead == "y" or goahead == "yes":
            rospy.loginfo("Motion plan approved. Execution starting.")
        else:
            rospy.loginfo("Motion execution cancelled.")
            return

    state.group.execute(grasp_in)
    rospy.sleep(2)

    state.close_gripper()

    # Move out from grasping
    waypoints = []
    waypoints.append(state.group.get_current_pose().pose)

    back_pose = copy.deepcopy(waypoints[0])
    back_pose.position.x -= 0.08
    back_pose.position.z += 0.1
    waypoints.append(back_pose)

    (grasp_out, frac) = state.group.compute_cartesian_path(waypoints,
                                                          0.01, 0.0,
                                                          avoid_collisions=False)

    if state.check_motion:
        goahead = raw_input("Is this plan okay? ")
        if goahead == "y" or goahead == "yes":
            rospy.loginfo("Motion plan approved. Execution starting.")
        else:
            rospy.loginfo("Motion execution cancelled.")
            return

    state.group.execute(grasp_out)

    state.home_arm()

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
    rospy.sleep(2)

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

        if self.check_motion:
            goahead = raw_input("Is this plan okay? ")
            if goahead == "y" or goahead == "yes":
                rospy.loginfo("Motion plan approved. Execution starting.")
            else:
                rospy.loginfo("Motion execution cancelled.")
                return

        self.group.go(wait=True)

    def close_gripper(self):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 10.0
        gripper_goal.command.position = 0.0

        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(4.0))

    def open_gripper(self):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 10.0
        gripper_goal.command.position = 0.1

        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(4.0))

    def move_to_xyz_target(self, target):
        adjusted_target = [target[0], target[1], target[2], 1]
        target_rpy = [0, -math.pi/4.0, 0]
        target_quat = rpy2quat(target_rpy)

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = target_quat[0]
        pose_target.orientation.y = target_quat[1]
        pose_target.orientation.z = target_quat[2]
        pose_target.orientation.w = target_quat[3]
        pose_target.position.x = adjusted_target[0]
        pose_target.position.y = adjusted_target[1]
        pose_target.position.z = adjusted_target[2]

        self.group.set_pose_target(pose_target)
        the_plan = self.group.plan()

        if self.check_motion:
            goahead = raw_input("Is this plan okay? ")
            if goahead == "y" or goahead == "yes":
                rospy.loginfo("Motion plan approved. Execution starting.")
            else:
                rospy.loginfo("Motion execution cancelled.")
                return

        self.group.go(wait=True)

    def __init__(self, check_motion_plans):
        self.action_state = robot_state.WAIT
        self.last_command_time = 0
        self.grabbed_object = 0
        self.check_motion = check_motion_plans

        self.obj_lock = Lock()
        self.perceived_objects = {}

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_max_velocity_scaling_factor(0.2)

        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()

        rospy.Subscriber("/rosie_arm_commands", RobotCommand, command_callback, callback_args=self)
        rospy.Subscriber("/rosie_observations", Observations, object_callback, callback_args=self)

        self.stats_pub = rospy.Publisher("/rosie_arm_status", RobotAction, queue_size=10)
        self.rate = rospy.Rate(10)

        self.close_gripper()
        while not rospy.is_shutdown():
            self.publish_status()
            self.rate.sleep()


# Note: fetch_moveit_config move_group.launch must be running
if __name__ == '__main__':
    rospy.init_node("rosie_motion_server")
    rospy.loginfo("Rosie motor node starting up...")

    human_motor_check = True

    for arg in sys.argv:
        if "check_motion_plans" in arg:
            words = arg.split('=')
            if words[1] == "true":
                rospy.loginfo("Human checks on motion plans will be required.")
            elif words[1] == "false":
                rospy.loginfo("Human checks on motion plans are disabled!")
                human_motor_check = False
            else:
                rospy.loginfo("Invalid check_motion_plans argument given.")
                sys.exit(0)

    time.sleep(15)
    status = robot_state(human_motor_check)
    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    group.get_move_action().cancel_all_goals()
