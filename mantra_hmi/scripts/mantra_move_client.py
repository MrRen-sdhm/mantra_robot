#!/usr/bin/env python

from __future__ import print_function
import rospy
from mantra_application.srv import *


def move_to_pose_named(pose_name):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('move_to_pose_named', timeout=5)
    print("[SRVICE] Found move to pose named service!")
    try:
        move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
        resp = move_to_pose_named(pose_name)
        print("[SRVICE] Move to pose named result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to pose named service call failed: %s" % e)
        return False


def move_to_pose_shift(axis, value):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('move_to_pose_shift', timeout=5)
    print("[SRVICE] Found move to pose shift service!")
    try:
        move_to_pose_shift = rospy.ServiceProxy('move_to_pose_shift', MoveToPoseShift)
        resp = move_to_pose_shift(axis, value)
        print("[SRVICE] Move to pose shift result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to pose shift service call failed: %s" % e)
        return False


def move_to_joint_states(joint_states):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('move_to_joint_states', timeout=5)
    print("[SRVICE] Found move to joint states service!")
    try:
        move_to_joint_states = rospy.ServiceProxy('move_to_joint_states', MoveToJointStates)
        resp = move_to_joint_states(joint_states)
        print("[SRVICE] Move to joint states result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to joint states service call failed: %s" % e)
        return False


def get_current_pose():
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('get_current_pose', timeout=5)
    print("[SRVICE] Found get current pose service!")
    try:
        get_current_pose = rospy.ServiceProxy('get_current_pose', GetCurrentPose)
        resp = get_current_pose()
        return resp.pose
    except rospy.ServiceException, e:
        print("[SRVICE] Get current pose service call failed: %s" % e)
        return False


def set_vel_scaling(scale):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('set_vel_scaling', timeout=5)
    print("[SRVICE] Found set vel scaling service!")
    try:
        set_vel_scaling = rospy.ServiceProxy('set_vel_scaling', SetVelScaling)
        resp = set_vel_scaling(scale)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Set vel scaling service call failed: %s" % e)
        return False


if __name__ == "__main__":
    print("[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home"))
    print("[SRVICE] get_current_pose result = ", get_current_pose())
    print("[SRVICE] move_to_joint_states result = %s" % move_to_joint_states([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))
    print("[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(0.2))
    print("[SRVICE] move_to_pose_shift result = %s" % move_to_pose_shift(2, -0.3))
    print("[SRVICE] get_current_pose result = ", get_current_pose())
    # print("[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(1.0))
    # print("[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home"))
