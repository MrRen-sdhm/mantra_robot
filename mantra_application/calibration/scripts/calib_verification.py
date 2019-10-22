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
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray, Bool
from tf.transformations import quaternion_from_euler
from transforms3d import quaternions


# 通过四元数计算方向向量
def cal_direction_vec(quat):
    quat_wxyz = (quat.w, quat.x, quat.y, quat.z)
    rotation_matrix = quaternions.quat2mat(quat_wxyz)
    direction_x = rotation_matrix[:,0] # 旋转矩阵第一列为x轴方向向量
    direction_y = rotation_matrix[:,1] # 旋转矩阵第二列为y轴方向向量
    direction_z = rotation_matrix[:,2] # 旋转矩阵第三列为z轴方向向量
    print "rotation_matrix:\n", rotation_matrix
    print "direction vector:\n", direction_x, direction_y, direction_z, "\n"
    return (direction_x, direction_y, direction_z)

# 通过方向向量及移动距离计算终点位姿, axis：0-x 1-y 2-z
def cal_pose_by_dir_vec(start_pose, dir_vec, distance, axis):
    end_pose = deepcopy(start_pose)
    end_pose.position.x += dir_vec[axis][0]*distance
    end_pose.position.y += dir_vec[axis][1]*distance
    end_pose.position.z += dir_vec[axis][2]*distance
    return end_pose

# 通过起点位姿及移动距离计算终点位姿, 为上面两个函数的整合 axis：0-x 1-y 2-z
def cal_end_pose_by_quat(start_pose, distance, axis):
    dir_vec = cal_direction_vec(start_pose.orientation)
    end_pose = cal_pose_by_dir_vec(start_pose, dir_vec, distance, axis)
    return end_pose

# ros格式四元数绕某轴旋转angle度，输入为[x,y,z,w]格式，输出也为[x,y,z,w]格式
def rot_around_axis(quat, angle, axis):
  if axis == 0:
    axis_vec = [1, 0, 0]
  elif axis == 1:
    axis_vec = [0, 1, 0]
  elif axis == 2:
    axis_vec = [0, 0, 1]

  quat_in = (quat[3], quat[0], quat[1], quat[2]) # [w,x,y,z]
  quat_trans = quaternions.axangle2quat(axis_vec, angle)
  quat_out = quaternions.qmult(quat_in, quat_trans)
  quat_out = (quat_out[1], quat_out[2], quat_out[3], quat_out[0])
  return quat_out

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
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        arm.set_max_velocity_scaling_factor(0.8)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_planning_time(0.5) # 规划时间限制为2秒
        arm.allow_replanning(False) # 当运动规划失败后，是否允许重新规划
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        self.group = arm
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander
        self.listener = tf.TransformListener()

    def lookup_trans(self):
      while not rospy.is_shutdown():
        try:
          (obj_trans, obj_quat) = self.listener.lookupTransform('/base_link', '/aruco_marker', rospy.Time(0))
          return (obj_trans, obj_quat)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          pass

    def go_to_pose_goal(self):
      group = self.group

      (obj_trans, obj_quat) = self.lookup_trans()

      step = 6
      for i in range(0, step+1):
        angle = 2*np.pi/step * i
        print "\n\n[INFO] Try angle:", angle

        (obj_trans, obj_quat) = self.lookup_trans()

        # 转换标记姿态，使z轴垂直于纸面
        quat_out = rot_around_axis(obj_quat, np.pi/2, 0)
        # 绕标记z轴旋转，保持机械臂z轴与标记垂直，尝试多种姿态
        quat_out = rot_around_axis(quat_out, angle, 2)

        # 目标为使机械臂穆端坐标系与转换后的标记坐标系重合
        print("[INFO] Try to plan a path to the aim pose once...")

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = self.reference_frame
        pose_goal.header.stamp = rospy.Time.now()

        pose_goal.pose.position.x = obj_trans[0]
        pose_goal.pose.position.y = obj_trans[1]
        pose_goal.pose.position.z = obj_trans[2]

        pose_goal.pose.orientation.x = quat_out[0]
        pose_goal.pose.orientation.y = quat_out[1]
        pose_goal.pose.orientation.z = quat_out[2]
        pose_goal.pose.orientation.w = quat_out[3]

        print "[INFO] Aim pose:\n", obj_trans
        print pose_goal.pose.orientation

        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        plan = group.go(wait=True)

        if plan:
          print "[INFO] Success!"
          break

      group.stop()
      group.clear_pose_targets()

    def aruco_trans(self):
      listener = tf.TransformListener()
      broadcaster = tf.TransformBroadcaster()
      while not rospy.is_shutdown():
        try:
          (obj_trans, obj_quat) = listener.lookupTransform('/base_link', '/aruco_marker', rospy.Time(0))
          rospy.loginfo("Pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", str(obj_trans), str(obj_quat))

          # 转换标记姿态，使z轴垂直于纸面
          quat_out = rot_around_axis(obj_quat, np.pi/2, 0)
          # 发布转换后的标记姿态
          broadcaster.sendTransform(obj_trans, # [x,y,z]
                                    quat_out,  # [x,y,z,w]
                                    rospy.Time.now(), "aruco_maker_trans", self.reference_frame)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass



if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    # mantra_pickup.group.set_named_target('test_2')
    # mantra_pickup.group.go()

    # 测试标记坐标转换
    # mantra_pickup.aruco_trans()

    print("Move to top of the object...")
    mantra_pickup.go_to_pose_goal()
    rospy.sleep(1)

    # print("Move to the init pose...")
    # mantra_pickup.group.set_named_target('pick_4')
    # mantra_pickup.group.go()
    # rospy.sleep(2)

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


