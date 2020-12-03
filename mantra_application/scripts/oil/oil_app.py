#!/usr/bin/env python
#coding=utf-8

import os
import sys
import copy
import math
import threading

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
import tf.transformations as tf_trans

import tf2_ros as tf2
import tf2_geometry_msgs as tf2_gm

from aruco_tag import ArUco


class OilApp(object):
    X = 0
    Y = 1
    Z = 2

    def __init__(self, tag_server):
        super(OilApp, self).__init__()
        self.ar_tag_track_ = tag_server

    def findArTag(self):
        # (is_find, position, orientation) = self.ar_tag_track_.find()

        pose_ar_tag = tf2_gm.PoseStamped()

        # 测试使用
        is_find = True
        # orientation = tf_trans.quaternion_from_euler(0, -math.pi / 3 * 2, 0)
        pose_ar_tag.pose.position.x = -0.128719220233
        pose_ar_tag.pose.position.y = 0.364502367234
        pose_ar_tag.pose.position.z = 1.1390057802

        pose_ar_tag.pose.orientation.x = 0.743059355513
        pose_ar_tag.pose.orientation.y = -0.493321566965
        pose_ar_tag.pose.orientation.z = 0.364886823662
        pose_ar_tag.pose.orientation.w = 0.267121815451

        # 实际使用
        # pose_ar_tag.pose.position.x = position[0]
        # pose_ar_tag.pose.position.y = position[1]
        # pose_ar_tag.pose.position.z = position[2]

        # pose_ar_tag.pose.orientation.x = orientation[0]
        # pose_ar_tag.pose.orientation.y = orientation[1]
        # pose_ar_tag.pose.orientation.z = orientation[2]
        # pose_ar_tag.pose.orientation.w = orientation[3]

        pose_ar_tag.header.frame_id = "base_link"

        return is_find, pose_ar_tag

    def translationPoseFromEnd(self, pose, direction, distance):
        '''
        # 以末端坐标系为基坐标系，沿着设定方向（x/y/z）移动一定距离
        # pose： 机械臂末端当前pose
        # direction: 沿着机械臂当前末端坐标系移动的方向
        # distance: 移动的距离
        '''
        transform2 = tf2.TransformStamped()
        transform2.transform.translation.x = pose.pose.position.x
        transform2.transform.translation.y = pose.pose.position.y
        transform2.transform.translation.z = pose.pose.position.z

        transform2.transform.rotation.x = pose.pose.orientation.x
        transform2.transform.rotation.y = pose.pose.orientation.y
        transform2.transform.rotation.z = pose.pose.orientation.z
        transform2.transform.rotation.w = pose.pose.orientation.w

        pose1 = tf2_gm.PoseStamped()
        if (direction == OilApp.X):
            pose1.pose.position.x = distance
        elif (direction == OilApp.Y):
            pose1.pose.position.y = distance
        elif (direction == OilApp.Z):
            pose1.pose.position.z = distance

        pose1.pose.orientation.w = 1

        pose2 = tf2_gm.do_transform_pose(pose1, transform2)

        pose2.header.frame_id = "world"

        return pose2

    def oilCartesianPath(self, move_group, direction=2, distance_in=0.09):
        ## 笛卡尔路径规划，加油动作
        # 设置路径点
        waypoints = []
        wpose = move_group.get_current_pose()

        step = 20

        for i in range(step + 1):
            # print(i)
            pose_in = self.translationPoseFromEnd(wpose, direction,
                                                  distance_in / step * i)
            waypoints.append(copy.deepcopy(pose_in.pose))

        # 根据路径点计算插补路径
        (plan,
         fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return (plan, fraction)

    def rotationFrameAxis(self, pose, axis, angle):
        transform2 = tf2.TransformStamped()
        transform2.transform.translation.x = pose.pose.position.x
        transform2.transform.translation.y = pose.pose.position.y
        transform2.transform.translation.z = pose.pose.position.z

        transform2.transform.rotation.x = pose.pose.orientation.x
        transform2.transform.rotation.y = pose.pose.orientation.y
        transform2.transform.rotation.z = pose.pose.orientation.z
        transform2.transform.rotation.w = pose.pose.orientation.w

        quaternion = [0] * 4
        if (axis == OilApp.X):
            quaternion = tf_trans.quaternion_from_euler(angle, 0, 0)
        elif (axis == OilApp.Y):
            quaternion = tf_trans.quaternion_from_euler(0, angle, 0)
        elif (axis == OilApp.Z):
            quaternion = tf_trans.quaternion_from_euler(0, 0, angle)

        pose1 = tf2_gm.PoseStamped()
        pose1.pose.orientation.x = quaternion[0]
        pose1.pose.orientation.y = quaternion[1]
        pose1.pose.orientation.z = quaternion[2]
        pose1.pose.orientation.w = quaternion[3]

        pose2 = tf2_gm.do_transform_pose(pose1, transform2)

        pose2.header.frame_id = "world"

        return pose2

    # def getGraspPose(self, pose):
    #     pose.
    #     euler = tf_trans.euler_from_quaternion([0, 0, 0, 1])
