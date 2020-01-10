#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi
from std_msgs.msg import String
from tf.transformations import *
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
        robot = moveit_commander.RobotCommander()
        
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
        # arm.set_named_target('home')
        # arm.go()

        self.box_name = ''
        self.scene = scene
        self.group = arm
        self.robot = robot
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander

        self.move_distance = 0.1
        self.back_distance = 0.15

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

    def plan_cartesian(self, waypoints, start_state=None, scale=1.0):
      group = self.group

      # 开始笛卡尔空间轨迹规划
      fraction = 0.0  # 路径规划覆盖率
      maxtries = 150  # 最大尝试规划次数
      attempts = 0  # 已经尝试规划次数

      # 设置机器臂运动初始状态
      if start_state is not None:
        state = self.robot.get_current_state()
        state.joint_state.position = list(start_state)
        group.set_start_state(state)
      else:
        group.set_start_state_to_current_state()      

      # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
      while fraction < 1.0 and attempts < maxtries:
          (plan, fraction) = group.compute_cartesian_path(
              waypoints,  # waypoint poses，路点列表
              0.005,  # eef_step，终端步进值
              0.0,  # jump_threshold，跳跃阈值
              True)  # avoid_collisions，避障规划

          # 尝试次数累加
          attempts += 1

          # 打印运动规划进程
          if attempts % 10 == 0:
              rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

      # 如果路径规划成功（覆盖率>90%）,则开始控制机械臂运动
      if fraction > 0.9:
        rospy.loginfo("Cartesian path computed successfully. Moving the arm.")
        plan = self.scale_trajectory_speed(plan, scale)
        if start_state is None:
          plan = group.execute(plan)
          rospy.loginfo("Cartesian path execution complete.")
          return True
        else:
          return plan

      # 如果路径规划失败，则打印失败信息
      else:
          rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
              maxtries) + " attempts.")
          return None

    def goto_pose_and_cartesian_continue(self):
      group = self.group

      pose_goal = geometry_msgs.msg.Pose()
      # pose_goal.header.frame_id = self.reference_frame
      # pose_goal.header.stamp = rospy.Time.now()


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

      print better_traj.joint_trajectory.points[-1].positions
      end_state = better_traj.joint_trajectory.points[-1].positions

      # cartesian
      pose_new = geometry_msgs.msg.PoseStamped()
      # pose_new.header.frame_id = self.reference_frame
      # pose_new.header.stamp = rospy.Time.now()
      pose_new.pose = cal_end_pose_by_quat(pose_goal.pose, 0.05, 2)

      waypoints = [pose_goal.pose, pose_new.pose]
      cartesian_plan = self.plan_cartesian(waypoints, start_state=end_state, scale=1.0)

      # if better_traj_len < 50:
      if better_traj_len < 50 and cartesian_plan is not None:
        plan = group.execute(better_traj, wait=True)

        # cartesian
        # pose_new = geometry_msgs.msg.PoseStamped()
        # pose_new.header.frame_id = self.reference_frame
        # pose_new.header.stamp = rospy.Time.now()
        # pose_new.pose = cal_end_pose_by_quat(pose_goal.pose, 0.05, 2)

        # waypoints = [pose_goal.pose, pose_new.pose]
        # cartesian_plan = self.plan_cartesian(waypoints, start_state=None, scale=1.0)
        plan = group.execute(cartesian_plan, wait=True)
      else:
        return False          

      # plan = group.go(wait=True)

      group.stop()
      group.clear_pose_targets()

      return True


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    ret = mantra_pickup.goto_pose_and_cartesian_continue()

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


