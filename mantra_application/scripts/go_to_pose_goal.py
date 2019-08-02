#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from transforms3d import quaternions


class MantraPickup:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param('~cartesian', True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(False)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0001)
        arm.set_goal_orientation_tolerance(0.0001)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        scene = moveit_commander.PlanningSceneInterface()

        print arm.get_current_pose(eef_link).pose
                                        
        # 控制机械臂运动到之前设置的姿态
        # arm.set_named_target('cali_1')
        # arm.go()

        self.box_name = ''
        self.scene = scene
        self.group = arm
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander

        self.move_distance = 0.1
        self.back_distance = 0.15

    def go_to_pose_goal(self):
      group = self.group

      pose_goal = geometry_msgs.msg.PoseStamped()
      pose_goal.header.frame_id = self.reference_frame
      pose_goal.header.stamp = rospy.Time.now()

      pose_goal.pose.position.x = 0.4
      pose_goal.pose.position.y = 0.0
      pose_goal.pose.position.z = 0.0

      euler = [pi, 0, pi/2]
      q = quaternion_from_euler(euler[0], euler[1], euler[2])
      pose_goal.pose.orientation.x = q[0]
      pose_goal.pose.orientation.y = q[1]
      pose_goal.pose.orientation.z = q[2]
      pose_goal.pose.orientation.w = q[3]

      # pose_goal.pose.orientation.x = -0.707246942408
      # pose_goal.pose.orientation.y = -0.706965263683
      # pose_goal.pose.orientation.z = 0.000940686011263
      # pose_goal.pose.orientation.w = 0.000996750072809

      group.set_start_state_to_current_state()
      group.set_pose_target(pose_goal, self.eef_link)

      plan = group.go(wait=True)
      group.stop()
      group.clear_pose_targets()

      return plan


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    ret = mantra_pickup.go_to_pose_goal()

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


