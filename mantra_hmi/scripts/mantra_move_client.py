#!/usr/bin/env python

from __future__ import print_function
import rospy
from mantra_application.srv import *


def test_all_service():
    if (move_to_pose_named(None, test=True) and move_to_pose_shift(None, None, test=True) and
        move_to_joint_states(None, test=True) and get_current_pose(test=True) and get_base_ee_link(test=True) and
            set_vel_scaling(None, test=True)):
        return True
    else:
        raise Exception("[ERROR] Service not started!")


def move_to_pose_named(pose_name, test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('move_to_pose_named', timeout=15)
        print("[SRVICE] Found move to pose named service!")
    except rospy.ROSException:
        print("[ERROR] Move to pose named service did not started!")
        return False

    if test:  # test success
        return True

    try:
        move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
        resp = move_to_pose_named(pose_name)
        print("[SRVICE] Move to pose named result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to pose named service call failed: %s" % e)
        return False


def move_to_pose_shift(axis, value, test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('move_to_pose_shift', timeout=15)
        print("[SRVICE] Found move to pose shift service!")
    except rospy.ROSException:
        print("[ERROR] Move to pose shift service did not started!")
        return False

    if test:  # test success
        return True

    try:
        move_to_pose_shift = rospy.ServiceProxy('move_to_pose_shift', MoveToPoseShift)
        resp = move_to_pose_shift(axis, value)
        print("[SRVICE] Move to pose shift result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to pose shift service call failed: %s" % e)
        return False


def move_to_joint_states(joint_states, test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('move_to_joint_states', timeout=15)
        print("[SRVICE] Found move to joint states service!")
    except rospy.ROSException:
        print("[ERROR] Move to joint states service did not started!")
        return False

    if test:  # test success
        return True

    try:
        move_to_joint_states = rospy.ServiceProxy('move_to_joint_states', MoveToJointStates)
        resp = move_to_joint_states(joint_states)
        print("[SRVICE] Move to joint states result:", resp.success)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Move to joint states service call failed: %s" % e)
        return False


def get_current_pose(test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('get_current_pose', timeout=15)
        print("[SRVICE] Found get current pose service!")
    except rospy.ROSException:
        print("[ERROR] Get current pose service did not started!")
        return False

    if test:  # test success
        return True

    try:
        get_current_pose = rospy.ServiceProxy('get_current_pose', GetCurrentPose)
        resp = get_current_pose()
        return resp.pose
    except rospy.ServiceException, e:
        print("[SRVICE] Get current pose service call failed: %s" % e)
        return False


def get_base_ee_link(test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('get_base_ee_link', timeout=15)
        print("[SRVICE] Found get base ee link service!")
    except rospy.ROSException:
        print("[ERROR] Get base ee link service did not started!")
        return False

    if test:  # test success
        return True

    try:
        get_base_ee_link = rospy.ServiceProxy('get_base_ee_link', GetBaseEELink)
        resp = get_base_ee_link()
        return resp.base_link, resp.ee_link
    except rospy.ServiceException, e:
        print("[SRVICE] Get base ee link call failed: %s" % e)
        return False


def set_vel_scaling(scale, test=False):
    print("[SRVICE] Wait for service ...")
    try:
        rospy.wait_for_service('set_vel_scaling', timeout=15)
        print("[SRVICE] Found set vel scaling service!")
    except rospy.ROSException:
        print("[ERROR] Set vel scaling service did not started!")
        return False

    if test:  # test success
        return True

    try:
        set_vel_scaling = rospy.ServiceProxy('set_vel_scaling', SetVelScaling)
        resp = set_vel_scaling(scale)
        return resp.success
    except rospy.ServiceException, e:
        print("[SRVICE] Set vel scaling service call failed: %s" % e)
        return False


if __name__ == "__main__":
    test_all_service()
    print("[SRVICE] get_end_effector result =", get_base_ee_link())
    print("[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home"))
    print("[SRVICE] get_current_pose result = ", get_current_pose())
    print("[SRVICE] move_to_joint_states result = %s" % move_to_joint_states([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))
    print("[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(0.2))
    print("[SRVICE] move_to_pose_shift result = %s" % move_to_pose_shift(2, -0.3))
    print("[SRVICE] get_current_pose result = ", get_current_pose())
    # print("[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(1.0))
    # print("[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home"))
