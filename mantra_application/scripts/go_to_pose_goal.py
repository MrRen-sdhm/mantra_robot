#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi
from std_msgs.msg import String
from tf.transformations import *
from transforms3d import quaternions

# ros格式四元数绕某轴旋转angle弧度，输入为[x,y,z,w]格式，输出也为[x,y,z,w]格式
def rot_around_axis(quat, angle, axis):
  if axis == 0:
    axis_vec = [1, 0, 0]
  elif axis == 1:
    axis_vec = [0, 1, 0]
  elif axis == 2:
    axis_vec = [0, 0, 1]

  # quat_in = (quat[3], quat[0], quat[1], quat[2]) # [w,x,y,z]
  # quat_trans = quaternions.axangle2quat(axis_vec, angle)
  # quat_out = quaternions.qmult(quat_in, quat_trans)
  # quat_out = (quat_out[1], quat_out[2], quat_out[3], quat_out[0])

  quat_out = quaternion_multiply(quat, quaternion_about_axis(pi, axis_vec))

  return quat_out

def pose_rot_around_axis(pose_in, angle, axis):
  quat_in = [pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w]
  quat_out = rot_around_axis(quat_in, angle, axis)

  pose_out = geometry_msgs.msg.PoseStamped()
  pose_out.header.frame_id = pose_in.header.frame_id
  pose_out.header.stamp = pose_in.header.stamp

  pose_out.pose.position = pose_in.pose.position

  pose_out.pose.orientation.x = quat_out[0]
  pose_out.pose.orientation.y = quat_out[1]
  pose_out.pose.orientation.z = quat_out[2]
  pose_out.pose.orientation.w = quat_out[3]

  return pose_out

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

        print "[INFO] Current pose:", arm.get_current_pose(eef_link).pose
                                        
        # 控制机械臂运动到之前设置的姿态
        # arm.set_named_target('pick_6')
        arm.set_named_target('home')
        arm.go()

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


# - Rotation: in Quaternion [0.978, 0.148, 0.140, -0.042]
#             in RPY (radian) [-3.099, -0.290, 0.294]
#             in RPY (degree) [-177.579, -16.593, 16.857]

# - Rotation: in Quaternion [-0.148, 0.978, 0.042, 0.140]
#             in RPY (radian) [3.099, 0.290, -2.847]
#             in RPY (degree) [177.572, 16.601, -163.139]


      # trans = [0.40699823412193165, -0.017990791602201735, 0.4216289524000239]
      # quat = [0.978191352021, 0.14801038459, 0.139602188532, -0.0417811735446]

# [INFO] Try pose: position: 
#   x: 0.41220727184781425
#   y: -0.08328811154879229
#   z: 0.5243095789390491
# orientation: 
#   x: 0.50251448694
#   y: 0.864447162785
#   z: -0.0140431986858
#   w: -0.00359365528417
# [ INFO] [1578297131.327199943]: ABORTED: Solution found but controller failed during execution
# [INFO] Try rotate 180: position: 
#   x: 0.41220727184781425
#   y: -0.08328811154879229
#   z: 0.5243095789390491
# orientation: 
#   x: 0.864447162785429
#   y: -0.5025144869403348
#   z: -0.003593655284172051
#   w: 0.014043198685836205

      trans = [0.41220727184781425, -0.08328811154879229, 0.5243095789390491]
      quat = [0.50251448694, 0.864447162785, -0.0140431986858, -0.00359365528417]

      # trans = [0.41220727184781425, -0.08328811154879229, 0.5243095789390491]
      # quat = [0.864447162785429, -0.5025144869403348, -0.003593655284172051, 0.014043198685836205]

      pose_goal.pose.position.x = trans[0]
      pose_goal.pose.position.y = trans[1]
      pose_goal.pose.position.z = trans[2]

      # euler = [pi, 0, pi/2]
      # q = quaternion_from_euler(euler[0], euler[1], euler[2])
      # pose_goal.pose.orientation.x = q[0]
      # pose_goal.pose.orientation.y = q[1]
      # pose_goal.pose.orientation.z = q[2]
      # pose_goal.pose.orientation.w = q[3]

      pose_goal.pose.orientation.x = quat[0]
      pose_goal.pose.orientation.y = quat[1]
      pose_goal.pose.orientation.z = quat[2]
      pose_goal.pose.orientation.w = quat[3]

      def find_better_traj(pose_goal):
        # get short traj
        traj_ls = []
        traj_len_ls = []
        for angle in [0, pi]:
          print "\n\n[INFO] Try angle:", angle

          # rotate around z
          pose_goal = pose_rot_around_axis(pose_goal, angle, 2)

          print "[INFO] Pose goal:", pose_goal
          print "[INFO] Euler:", euler_from_quaternion([pose_goal.pose.orientation.x,  pose_goal.pose.orientation.y,  pose_goal.pose.orientation.z,  pose_goal.pose.orientation.z])

          group.set_start_state_to_current_state()
          group.set_pose_target(pose_goal, self.eef_link)
          traj = group.plan()

          traj_len  = len(traj.joint_trajectory.points)
          print "[INFO] Traj len:", traj_len

          traj_ls.append(traj)
          traj_len_ls.append(traj_len)

        better_traj_idx = traj_len_ls.index(min(traj_len_ls))
        better_traj = traj_ls[better_traj_idx]
        better_traj_len = traj_len_ls[better_traj_idx]

        print "[INFO] Better traj index:", better_traj_idx
        print "[INFO] Better traj len:", better_traj_len

        return better_traj, better_traj_len

      better_traj, better_traj_len = find_better_traj(pose_goal)

      if (better_traj_len < 50):
        plan = group.execute(better_traj, wait=True)
      else:
        return False          

      # plan = group.go(wait=True)

      group.stop()
      group.clear_pose_targets()

      return True


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


