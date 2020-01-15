#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import thread
import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
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
    # print "rotation_matrix:\n", rotation_matrix
    # print "direction vector:\n", direction_x, direction_y, direction_z, "\n"
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

# ros格式四元数绕某轴旋转angle弧度，输入为[x,y,z,w]格式，输出也为[x,y,z,w]格式
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
        arm.set_goal_position_tolerance(0.00001)
        arm.set_goal_orientation_tolerance(0.00001)

        arm.set_max_velocity_scaling_factor(0.5)
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
        self.broadcaster = tf.TransformBroadcaster()

    def publish_trans(self, obj_trans, obj_quat):
      try:
        # 发布转换后的标记姿态  需要在rviz中设置TF的Frame Timeout为30s
        for i in range(5000):
          self.broadcaster.sendTransform(obj_trans, # [x,y,z]
                                    obj_quat,  # [x,y,z,w]
                                    rospy.Time.now(), "aruco_maker_trans", self.reference_frame)
          # print "===================", obj_trans
      except:
        pass

    def lookup_trans(self):
      print "[INFO] Lookup trans between base_link and aruco_marker..."
      while not rospy.is_shutdown():
        try:
          # self.listener.waitForTransform('/base_link', '/aruco_marker', rospy.Time(), rospy.Duration(1.0))
          (obj_trans, obj_quat) = self.listener.lookupTransform('/base_link', '/aruco_marker', rospy.Time(0))
          return (obj_trans, obj_quat)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          pass

    @staticmethod
    def scale_trajectory_speed(traj, scale):
      new_traj = RobotTrajectory()
      new_traj.joint_trajectory = traj.joint_trajectory

      n_joints = len(traj.joint_trajectory.joint_names)
      n_points = len(traj.joint_trajectory.points)
      points = list(traj.joint_trajectory.points)

      for i in range(n_points):
          point = JointTrajectoryPoint()
          point.positions = traj.joint_trajectory.points[i].positions

          point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
          point.velocities = list(traj.joint_trajectory.points[i].velocities)
          point.accelerations = list(traj.joint_trajectory.points[i].accelerations)

          for j in range(n_joints):
              point.velocities[j] = point.velocities[j] * scale
              point.accelerations[j] = point.accelerations[j] * scale * scale
          points[i] = point

      new_traj.joint_trajectory.points = points
      return new_traj

    def move_cartesian(self, waypoints, scale):
      group = self.group
      # 开始笛卡尔空间轨迹规划
      fraction = 0.0  # 路径规划覆盖率
      maxtries = 10  # 最大尝试规划次数
      attempts = 0  # 已经尝试规划次数

      # 设置机器臂当前的状态作为运动初始状态
      group.set_start_state_to_current_state()

      # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
      while fraction < 1.0 and attempts < maxtries:
          (plan, fraction) = group.compute_cartesian_path(
              waypoints,  # waypoint poses，路点列表
              0.01,  # eef_step，终端步进值
              0.0,  # jump_threshold，跳跃阈值
              True)  # avoid_collisions，避障规划

          # 尝试次数累加
          attempts += 1

          # 打印运动规划进程
          if attempts % 10 == 0:
              rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

      # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
      if fraction == 1.0:
          rospy.loginfo("Path computed successfully. Moving the arm.")
          plan = self.scale_trajectory_speed(plan, 0.2)
          group.execute(plan)
          rospy.loginfo("Path execution complete.")
      # 如果路径规划失败，则打印失败信息
      else:
          rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
              maxtries) + " attempts.")

    def change_ee_joint_state(self, angle):
      group = self.group

      # Planning to a Joint Goal
      joint_goal = group.get_current_joint_values()
      joint_goal[6] = joint_goal[6] + angle

      group.go(joint_goal, wait=True)

      group.stop()

    def go_to_pose_goal(self):
      group = self.group

      print "[INFO] Lookup for marker trans..."
      (obj_trans, obj_quat) = self.lookup_trans()

      step = 6
      for i in range(0, step+1):
        angle = 2*np.pi/step * i
        print "\n\n[INFO] Try angle:", angle

        (obj_trans, obj_quat) = self.lookup_trans()

        print "[INFO] Marker loc:", (obj_trans, obj_quat)

        # 转换标记姿态，使z轴垂直于纸面向里
        quat_out = rot_around_axis(obj_quat, np.pi/2, 0)
        # 绕标记z轴旋转，保持机械臂z轴与标记垂直，尝试多种姿态
        quat_out = rot_around_axis(quat_out, angle, 2)

        # 目标为使机械臂末端坐标系与转换后的标记坐标系重合
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

        print "pose_goal before", pose_goal
        pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -0.1, 2)
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        plan = group.go(wait=True)

        # 与纸面距离20cm
        pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, 0.08, 2)
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        plan = group.go(wait=True)

        print "[INFO] Aim pose:\n", pose_goal

        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        traj = group.plan()
        len(traj.joint_trajectory.points)
        group.execute(traj)
        if len(traj.joint_trajectory.points) > 0:
          print "[INFO] Success!"
          break

        # plan = group.go(wait=True)
        # if plan:
        #   print "[INFO] Success!"
        #   break

      group.stop()
      group.clear_pose_targets()

    def go_to_pose(self):
      group = self.group

      (obj_trans, obj_quat) = self.lookup_trans()

      self.publish_trans(obj_trans, obj_quat)

      print "[INFO] Marker loc:", (obj_trans, obj_quat)

      # 转换标记姿态，使z轴垂直于纸面向里
      quat_out = rot_around_axis(obj_quat, np.pi/2, 0)
      # 绕标记z轴旋转，保持机械臂z轴与标记垂直，尝试多种姿态
      quat_out = rot_around_axis(quat_out, np.pi/2, 2)  # np.pi/2

      # 目标为使机械臂末端坐标系与转换后的标记坐标系重合
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

      # 退回10cm
      pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -0.057, 2)
      # pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -0.15, 2)

      group.set_start_state_to_current_state()
      group.set_pose_target(pose_goal)
      group.go()

      print "Step1:", pose_goal.pose

      rospy.loginfo("Step1 success!!!")

      exit()
      time.sleep(0.5)

      ##############   笛卡尔轨迹规划    #################
      # 初始化路点列表
      waypoints = []

      # 按末端坐标系方向向量平移, 计算终点位姿
      wpose = cal_end_pose_by_quat(pose_goal.pose, 0.1, 2)
      waypoints.append(deepcopy(wpose))

      self.move_cartesian(waypoints, 0.5)

    def aruco_loc_show(self):
      listener = tf.TransformListener()
      broadcaster = tf.TransformBroadcaster()
      while not rospy.is_shutdown():
        try:
          (obj_trans, obj_quat) = listener.lookupTransform('/base_link', '/aruco_marker', rospy.Time(0))
          rospy.loginfo("Pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", str(obj_trans), str(obj_quat))

          # 转换标记姿态，使z轴垂直于纸面
          quat_out = rot_around_axis(obj_quat, np.pi/2, 0)

          # 后退20cm
          pose_goal = geometry_msgs.msg.PoseStamped()
          pose_goal.pose.position.x = obj_trans[0]
          pose_goal.pose.position.y = obj_trans[1]
          pose_goal.pose.position.z = obj_trans[2]

          pose_goal.pose.orientation.x = quat_out[0]
          pose_goal.pose.orientation.y = quat_out[1]
          pose_goal.pose.orientation.z = quat_out[2]
          pose_goal.pose.orientation.w = quat_out[3]

          # 与纸面距离20cm
          pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -0.2, 2)

          obj_trans = [pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z]
          quat_out = [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w]
          
          # 发布转换后的标记姿态
          broadcaster.sendTransform(obj_trans, # [x,y,z]
                                    quat_out,  # [x,y,z,w]
                                    rospy.Time.now(), "aruco_maker_trans", self.reference_frame)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass



if __name__ == "__main__":
  try:
    print("[INFO] Start calib verification...")

    mantra_pickup = MantraPickup()

    # points = ['cali_3', 'fusion_left2', 'fusion_left1', 'fusion_1', 'fusion_right1', 'fusion_right2']
    # points = ['cali_3', 'fusion_left', 'fusion_1', 'fusion_right']
    # points = ["pick_8", "fusion_1", "fusion_2", "pick_8"]
    # points = ["fusion_new_left", "fusion_new_left_rot", "fusion_middle", "fusion_new_right_rot", "fusion_new_right"]

    points = ["fusion_1", "fusion_2", "fusion_3"]
    # near
    # points = ["capture_l1", "capture_m1", "capture_r1", "capture_r2", "capture_m2", "capture_l2", "capture_l3", "capture_m3", "capture_r3"]
    # close up
    # points = ["capture_l1", "capture_r1"]

    
    
    mantra_pickup.group.set_named_target("test_4")
    plan = mantra_pickup.group.go()
    if not plan:
      exit()

    for point in points:
      print "[INFO] Move to point", point
      mantra_pickup.group.set_named_target(point)
      plan = mantra_pickup.group.go()
      if not plan:
        break

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


