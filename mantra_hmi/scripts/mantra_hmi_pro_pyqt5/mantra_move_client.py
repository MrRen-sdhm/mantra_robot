#!/usr/bin/env python

import rospy
from mantra_application.srv import *


def move_to_pose_named(pose_name):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('move_to_pose_named', timeout=5)
    try:
        move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
        resp = move_to_pose_named(pose_name)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Move to pose named service call failed: %s" % e


def move_to_joint_states(joint_states):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('move_to_joint_states', timeout=5)
    try:
        move_to_joint_states = rospy.ServiceProxy('move_to_joint_states', MoveToJointStates)
        resp = move_to_joint_states(joint_states)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Move to joint states service call failed: %s" % e


def set_vel_scaling(scale):
    print("[SRVICE] Wait for service ...")
    rospy.wait_for_service('set_vel_scaling', timeout=5)
    try:
        set_vel_scaling = rospy.ServiceProxy('set_vel_scaling', SetVelScaling)
        resp = set_vel_scaling(scale)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Set vel scaling service call failed: %s" % e


if __name__ == "__main__":
    print "[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home")
    print "[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(0.1)
    print "[SRVICE] move_to_joint_states result = %s" % move_to_joint_states([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    print "[SRVICE] set_vel_scaling result = %s" % set_vel_scaling(1.0)
    print "[SRVICE] move_to_pose_named result = %s" % move_to_pose_named("home")
