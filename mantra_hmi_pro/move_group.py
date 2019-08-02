#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  :

import sys
import rospy
import copy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list


class MoveGroup(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroup, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object.
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_max_velocity_scaling_factor(0.5)
        group.set_max_acceleration_scaling_factor(0.5)

        # 当运动规划失败后，允许重新规划
        # group.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        group.set_goal_position_tolerance(0.05)
        group.set_goal_orientation_tolerance(0.05)

        # Getting Basic Information
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print ("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (robot.get_current_state())
        print ("")

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def set_vel_scaling(self, scale):
        self.group.set_max_velocity_scaling_factor(scale)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene

        # Ensuring Collision Updates Are Receieved
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def go_to_joint_state(self, goal_positions):
        group = self.group
        ret = False

        # Planning to a Joint Goal
        ret = group.go(goal_positions, wait=True)
        # try:
        #     ret = group.go(goal_positions, wait=True)
        # except moveit_commander.MoveItCommanderException:
        #     print (moveit_commander.MoveItCommanderException)

        group.stop()

        # current_joints = self.group.get_current_joint_values()

        return ret

    def go_to_joint_state_0(self):
        group = self.group

        # Planning to a Joint Goal
        joint_goal = group.get_current_joint_values()
        # joint_goal[0] = pi/4
        # joint_goal[1] = pi/8
        # joint_goal[2] = pi/8
        # joint_goal[3] = pi/16
        # joint_goal[4] = pi/32
        # joint_goal[5] = pi/64
        # joint_goal[6] = pi/64
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = pi

        group.go(joint_goal, wait=True)

        group.stop()

        current_joints = self.group.get_current_joint_values()

    def go_to_joint_state_1(self):
        group = self.group

        # Planning to a Joint Goal
        joint_goal = group.get_current_joint_values()
        # joint_goal[0] = -pi/4
        # joint_goal[1] = -pi/8
        # joint_goal[2] = -pi/8
        # joint_goal[3] = -pi/16
        # joint_goal[4] = -pi/32
        # joint_goal[5] = -pi/64
        # joint_goal[6] = -pi/64

        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = -pi

        group.go(joint_goal, wait=True)

        group.stop()

        current_joints = self.group.get_current_joint_values()

    def go_to_pose_named(self, pose_name):
        group = self.group

        group.set_named_target(pose_name)
        group.go()
        group.stop()

    def go_to_pose_goal(self):
        group = self.group

        # We can plan a motion for this group to a desired pose for the end-effector:
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.header.stamp = rospy.Time.now()

        pose_goal.pose.position.x = 0.500
        pose_goal.pose.position.y = 0.000
        pose_goal.pose.position.z = 0.500

        euler = [0, 0, 0]
        # euler = [0, -pi, 0]
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        plan = group.go(wait=True)
        group.stop()

        # traj = group.plan()
        # print "============ Press `Enter` to execute ..."
        # raw_input()
        # group.execute(traj)
        # group.stop()

        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose


def main():
    try:
        tutorial = MoveGroup()

        tutorial.go_to_joint_state_0()
        rospy.sleep(0.5)
        tutorial.go_to_joint_state_1()
        rospy.sleep(0.5)
        tutorial.go_to_pose_named("home")

        print ("============ Complete! ============")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
