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
from oil_app import OilApp

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('oil_app', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_max_velocity_scaling_factor(0.4)  # 设置最大关节速度
    # move_group.set_goal_joint_tolerance(0.1)

    tag_server = ArUco("base_link", "aruco_marker_frame")
    oil = OilApp(tag_server)

    # 回到home
    move_group.set_named_target("home")
    if (not move_group.go()):
        rospy.logerr("home is go filed")
        sys.exit()
    move_group.clear_pose_targets()
    rospy.sleep(2)

    # 设置为寻找二维码时的关节
    move_group.set_named_target("oil_look")
    if (not move_group.go()):
        rospy.logerr("oil_look is go filed")
        sys.exit()
    move_group.clear_pose_targets()
    rospy.sleep(2)

    # 识别二维码
    is_find, pose_ar_tag = oil.findArTag()
    if (not is_find):
        print(" \033[1;31m ArTag is not find \033[0m")
        sys.exit()
    print("[findArTag] pose", pose_ar_tag)

    # 将二维码坐标系转换坐标到加油口
    pose_oil = oil.translationPoseFromEnd(pose_ar_tag, OilApp.Y, 0.01)
    print("[transformPoseFromEnd] pose_oil", pose_oil)

    # 沿着加油口坐标系 后退一定距离作为加油起始点
    pose_out = oil.translationPoseFromEnd(pose_oil, OilApp.Z, 0.25)
    print("[transformPoseFromEnd] pose_out", pose_out)

    # 翻转使得z轴朝外,即末端夹爪坐标系z轴
    pose_out_turn = oil.rotationFrameAxis(pose_out, OilApp.Y, math.pi)
    move_group.set_pose_target(pose_out_turn)
    if (not move_group.go()):
        print(" \033[1;31m pose_out_turn move_group.go() is failed \033[0m")
        sys.exit()

    # ## 笛卡尔路径规划，加油动作
    # (plan, fraction) = oil.oilCartesianPath(move_group,
    #                                         direction=OilApp.Z,
    #                                         distance_in=0.1)

    # if (fraction >= 0.9):
    #     print("compute_cartesian_path fraction:", fraction)
    #     move_group.execute(plan)
    # else:
    #     rospy.logerr("compute_cartesian_path is failed")

    # pose_oil = move_group.get_current_pose()

    # pose_out = oil.translationPoseFromEnd(pose_oil, OilApp.Z, -0.15)
    # move_group.set_pose_target(pose_out)
    # if (not move_group.go()):
    #     rospy.logerr("pose_out is go filed")
    #     sys.exit()
    # print("[transformPoseFromEnd] pose_out", pose_out)
    # rospy.sleep(2)

    # move_group.set_max_velocity_scaling_factor(0.1)  # 设置最大关节速度
    # if (fraction >= 0.9):
    #     print("compute_cartesian_path fraction:", fraction)
    #     move_group.execute(plan)
    # else:
    #     rospy.logerr("compute_cartesian_path is failed")

    # # 识别二维码
    # is_find, pose_ar_tag = oil.findArTag()
    # if (not is_find):
    #     print(" \033[1;31m ArTag is not find \033[0m")
    #     sys.exit()
    # print("[findArTag] pose", pose_ar_tag)

    # # 将二维码坐标系转换坐标到加油口
    # pose_oil = oil.translationPoseFromEnd(pose_ar_tag, OilApp.Y, 0.1)
    # print("[transformPoseFromEnd] pose_oil", pose_oil)

    # # 沿着加油口坐标系 后退一定距离作为加油起始点
    # pose_out = oil.translationPoseFromEnd(pose_oil, OilApp.Z, 0.1)
    # print("[transformPoseFromEnd] pose_out", pose_out)

    # # 翻转使得x轴朝外,即末端夹爪坐标系x轴
    # pose_out_turn = oil.rotationFrameAxis(pose_out, OilApp.Y, math.pi / 2)
    # move_group.set_pose_target(pose_out_turn)
    # if (not move_group.go()):
    #     print(" \033[1;31m pose_out move_group.go() is failed \033[0m")
    #     sys.exit()

    # ## 笛卡尔路径规划，加油动作
    # (plan, fraction) = oil.oilCartesianPath(move_group, distance_in=0.1)

    # if (fraction >= 0.9):
    #     print("compute_cartesian_path fraction:", fraction)
    #     move_group.execute(plan)
    # else:
    #     rospy.logerr("compute_cartesian_path is failed")
