#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  :

from __future__ import print_function

import sys
import threading
import rospy
import copy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from math import pi
from tf.transformations import quaternion_from_euler

goal_pose_vel = [0]*8
move_command = [0]*3
move_group_state = False


class MoveGroup(object):

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

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()

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

        return plan


def goal_pose_vel_callback(data):
    global goal_pose_vel
    # print("[Goal]", data.data)
    goal_pose_vel = data.data


def move_command_callback(data):
    global move_command
    # if data.data[0] != [0, 0, 0]:
    #     print("[Cmd]", data.data)
    move_command = data.data


def main():
    global move_group_state, goal_pose_vel
    rospy.init_node('mantra_move')
    try:
        move_group = MoveGroup()
        rospy.Subscriber("mantra_goal_pose_vel", Float32MultiArray, goal_pose_vel_callback)
        rospy.Subscriber("mantra_move_command", Int32MultiArray, move_command_callback)
        move_state_pub = rospy.Publisher('mantra_move_state', Bool, queue_size=1)

        r = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            move_group_state = False  # 状态复位
            if move_command[0] == 1:  # 关节运动
                move_group_state = True
                move_group.go_to_joint_state(goal_pose_vel[0:7])
                print("[INFO] Go to joint state done.")
            if move_command[1] == 1:  # 回零
                move_group_state = True
                move_group.go_to_pose_named('home')
                print("[INFO] Back home done.")
            if move_command[2] == 1:  # 调速
                move_group_state = True
                move_group.set_vel_scaling(goal_pose_vel[7])
                print("[INFO] Change speed done.")
                
            move_state_pub.publish(move_group_state)  # 发布执行状态
            
            r.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
