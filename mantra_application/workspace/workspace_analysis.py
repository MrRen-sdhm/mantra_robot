#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi
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


class MantraPickup:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
                        
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
        arm.set_max_velocity_scaling_factor(1.0)

        arm.set_planning_time(0.05) # 规划时间限制
        
        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        print "[INFO] Current pose:", arm.get_current_pose(eef_link).pose
                                        
        # 控制机械臂运动到之前设置的姿态
        arm.set_named_target('work')
        arm.go()

        self.group = arm
        self.robot = robot
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander

    # ############## 末端z轴笛卡尔路径规划  ##############
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

    def plan_cartesian(self, waypoints, scale=1.0, start_state=None):
      group = self.group

      # 开始笛卡尔空间轨迹规划
      fraction = 0.0  # 路径规划覆盖率
      maxtries = 2  # 最大尝试规划次数
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
          if attempts % 2 == 0:
              rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

      # 如果路径规划成功（覆盖率>90%）,则开始控制机械臂运动
      if fraction > 0.9:
        rospy.loginfo("Cartesian path computed successfully.")
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

    def cartesian_move(self, start_state, target_point, dis, scale):
      group = self.group

      pose_goal = geometry_msgs.msg.PoseStamped()
      pose_goal.header.frame_id = self.reference_frame
      pose_goal.header.stamp = rospy.Time.now()

      trans = target_point

      pose_goal.pose.position.x = trans[0]
      pose_goal.pose.position.y = trans[1]
      pose_goal.pose.position.z = trans[2]

      euler = [pi, 0, pi/2]
      q = quaternion_from_euler(euler[0], euler[1], euler[2])
      pose_goal.pose.orientation.x = q[0]
      pose_goal.pose.orientation.y = q[1]
      pose_goal.pose.orientation.z = q[2]
      pose_goal.pose.orientation.w = q[3]

      # 前进
      waypoints = []
      wpose = pose_goal.pose
      waypoints.append(deepcopy(wpose))

      wpose = cal_end_pose_by_quat(wpose, dis, 2)
      waypoints.append(deepcopy(wpose))

      plan = self.plan_cartesian(waypoints, scale=scale, start_state=start_state)

      if plan:
        result = len(plan.joint_trajectory.points)  # 轨迹点数不等于0则规划成功
      else:
        return False

      result = len(plan.joint_trajectory.points)  # 轨迹点数不等于0则规划成功

      if result: return True
      else: return False

    def go_to_pose_goal(self, target_point, exe=False):
      start = time.time()

      group = self.group

      pose_goal = geometry_msgs.msg.PoseStamped()
      pose_goal.header.frame_id = self.reference_frame
      pose_goal.header.stamp = rospy.Time.now()

      # trans = [0.563, -0.000, 0.150]
      trans = target_point

      pose_goal.pose.position.x = trans[0]
      pose_goal.pose.position.y = trans[1]
      pose_goal.pose.position.z = trans[2]

      euler = [pi, 0, pi/2]
      q = quaternion_from_euler(euler[0], euler[1], euler[2])
      pose_goal.pose.orientation.x = q[0]
      pose_goal.pose.orientation.y = q[1]
      pose_goal.pose.orientation.z = q[2]
      pose_goal.pose.orientation.w = q[3]

      # 设置初始位置为零点
      state = self.robot.get_current_state()
      # state = RobotState()
      # print state.joint_state.position
      if not exe:
        state.joint_state.position = [0,0,0,0,0,0,0]
      group.set_start_state(state)

      # print "time1", time.time() - start

      # 设置目标位置
      group.set_pose_target(pose_goal, self.eef_link)
      plan = group.plan() # 规划运动路径

      # print "time2", time.time() - start

      if exe:
        group.execute(plan, wait=True)

      if plan:
        result = len(plan.joint_trajectory.points)  # 轨迹点数不等于0则规划成功
      else: return False, None

      if result:
        return True, plan.joint_trajectory.points[-1].positions  # 返回成功与否，及最后一个轨迹点对应的关节角
      else:
        return False, None


# 3D可视化显示所有点
def plot_points_3d(reachable_points, unreachable_points):
  fig=plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(reachable_points[:,0], reachable_points[:,1], reachable_points[:,2], c='r', s=15.5, linewidth=0, alpha=1, marker=".")
  # ax.scatter(unreachable_points[:,0], unreachable_points[:,1], unreachable_points[:,2], c='b', s=15.5, linewidth=0, alpha=1, marker=".")

  # plt.title('Point Cloud')
  # ax.axis('equal')  # {equal, scaled}
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.show()


# 分析可达点云
def analysis_z(xyz_ls, reachable_points, mode):
  z_ls = xyz_ls[2]
  points_z_ls = extract_points_z_ls(z_ls, reachable_points)
  rects = find_max_rect(points_z_ls, xyz_ls)
  plot_points_per_z(points_z_ls, rects, mode)
  plot_pointnum_area_per_z(points_z_ls, rects, xyz_ls, mode)


# 寻找最大面积矩形
def find_max_rect(points_z_ls, xyz_ls):
  rects = []
  for i, it in enumerate(points_z_ls):
    z = it[0]
    points_z = np.array(it[1])

    points_y0 = []
    for point in points_z:
      if abs(point[1]) < 1e-4: # 找到y=0的所有点
        points_y0.append(point)

    if len(points_y0) > 0:
      x_min = np.min(np.array(points_y0), axis=0)[0] # x最小值

      start_point = [x_min, 0] # 查找起点，即内抛物线最高点，也即矩形左下角点

      # 矩形右上角点处于外抛物线上，查找在外抛物线上的点
      x_ls, y_ls = xyz_ls[0], xyz_ls[1]
      x_ls = x_ls[::-1] # 倒序，从大到小遍历
      y_ls = y_ls[y_ls >= 0] # 仅遍历y大于等于0的点即可，对称

      def find_point(x, y, points): # 查找是否存在满足xy坐标的点
        for point in points_z:
          if abs(x - point[0]) < 1e-4 and abs(y - point[1]) < 1e-4:
            return True
        return False
      
      end_points = []
      for y in y_ls:
        for x in x_ls:
          if x < x_min:  # 左侧点无需查找
            continue

          if find_point(x, y, points_z):
            end_points.append([x,y])
            break

      # 计算各矩形面积
      area_ls = []
      for end_point in end_points:
        area = (end_point[1] - start_point[1]) * (end_point[0] - start_point[0])
        area_ls.append(area)

      max_area_idx = np.argmax(area_ls)  # 最大面积索引
      max_area = area_ls[max_area_idx]  # 最大面积
      max_area_end_point = end_points[max_area_idx]  # 最大面积对应的右上角点
      w = max_area_end_point[1] - start_point[1]
      h = max_area_end_point[0] - start_point[0]

      rects.append([start_point, max_area_end_point])
    else:
      rects.append(None)
  
  return rects
    

# 按z划分点云
def extract_points_z_ls(z_ls, reachable_points):
  points_z_ls = []
  for i, z in enumerate(z_ls):
    # if z < 0.2:
    #   continue

    points_z = []
    for point in reachable_points:
      if abs(z - point[2]) < 1e-4:
        points_z.append(point)

    points_z_ls.append([z, points_z])

  return points_z_ls

# 二维可视化每个z对应的点云
def plot_points_per_z(points_z_ls, rects, mode):
  fig = plt.figure(figsize=(12, 9))
  plt.subplots_adjust(wspace=0.05, hspace=0.05) # 设置子图间距

  for i, it in enumerate(points_z_ls):
    z = it[0]
    points_z = np.array(it[1])
    
    ax = fig.add_subplot(3,4,i+1)
    # ax.set_title('z=%.3f' % z)
    plt.text(0.45, -0.6, 'z=%.2f' % z, size=11, color="k")

    ax.set_xlim(-0.05, 0.7)
    ax.set_ylim(-0.65, 0.65)

    # 隐藏部分坐标轴，仅全图左侧和底部显示坐标轴label
    if i <= 7: ax.set_xticks([])
    else: plt.xlabel('X(m)')
    if i % 4 != 0: ax.set_yticks([])
    else: plt.ylabel('Y(m)')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    if points_z.shape[0] > 0:
      ax.scatter(x=points_z[:,0],y=points_z[:,1],c = 'r',marker = '.',label = '%d' % points_z.shape[0])
      ax.legend(prop={'size':9})
    else: plt.text(0.2, 0, 'No Points' % area, size=11, color="k")

    # 绘制矩形
    if rects[i]:
      start_point, end_point = rects[i]
      w = end_point[1] - start_point[1]
      h = end_point[0] - start_point[0]
      # ax.plot(end_point[0], end_point[1], marker='*')
      area = 2 * w * h

      # 对称
      start_point_new = [start_point[0], start_point[1] - w]
      w = 2*w
      ax.add_patch(patches.Rectangle(start_point_new, h, w, color = 'deepskyblue', alpha = 0.5)) # 左下起点，长，宽，颜色，α

      # plt.text(0, 0, 'area=%.2f' % area, size=11, color="k")

      if i <= 7:
        plt.annotate(s='area=%.2f' % area,xy=start_point,xytext=(0,-0.02),size=10,color='k',rotation=0,\
                arrowprops=dict(arrowstyle='-|>',connectionstyle='arc3',color='green'),\
                bbox=dict(boxstyle='round,pad=0.4', fc='white', ec='k',lw=0.5,alpha=0.4))
      else:
        plt.annotate(s='area=%.2f' % area,xy=(end_point[0],0),xytext=(0.61,0.155),size=10,color='k',rotation=90,\
                arrowprops=dict(arrowstyle='-|>',connectionstyle='arc3',color='green'),\
                bbox=dict(boxstyle='round,pad=0.4', fc='white', ec='k',lw=0.5,alpha=0.4))

  plt.savefig('workspace_analysis_mode%d.svg' % mode, dpi=600, bbox_inches='tight', pad_inches=0.0)
  plt.show()


def plot_pointnum_area_per_z(points_z_ls, rects, xyz_ls, mode):
  z_ls = xyz_ls[2]
  pointnum_ls = []
  area_ls = []
  for i, it in enumerate(points_z_ls):
    points_z = it[1]
    pointnum_ls.append(len(points_z))

    # 计算面积
    if rects[i]:
      start_point, end_point = rects[i]
      w = end_point[1] - start_point[1]
      h = end_point[0] - start_point[0]
      # ax.plot(end_point[0], end_point[1], marker='*')
      area = 2 * w * h
    else: area = 0
    
    area_ls.append(area)

  print "pointnum_ls:", pointnum_ls
  print "area_ls:", area_ls

  fig = plt.figure()
  ax1 = fig.add_subplot(111)
  line1 = ax1.plot(z_ls,pointnum_ls,'r*-',label='number of points')
  # ax2.set_ylim([0, np.e])
  ax1.set_ylabel('number of points', size=12)
  # ax1.set_title('')
  ax1.set_xlabel('Z(m)', size=13)

  ax2 = ax1.twinx()  # this is the important function
  line2 = ax2.plot(z_ls,area_ls,'m.-',label='area of rectangles')
  # ax2.set_ylim([0, np.e])
  ax2.set_ylabel('area of rectangles', size=12)
  # ax2.set_xlabel('')

  # 合并图例
  lines = line1+line2
  labels = [l.get_label() for l in lines]
  ax1.legend(lines, labels, loc = 6)

  plt.savefig('pointnum_area_per_z_mode%d.svg' % mode, dpi=600, bbox_inches='tight', pad_inches=0.0)
  plt.show()


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    mode = 1  # 0：仅运动到目标位置  1：运动到目标位置后还进行笛卡尔直线规划
    cartesian_dis = 0.05  # 笛卡尔直线运动距离

    # #################################   运动规划测试   ##################################
    if 0:
      target_point = [0.563, -0.000, 0.150]
      if mode == 1: target_point[2] += cartesian_dis  # 预留出直线轨迹长度

      start = time.time()
      success, end_state = mantra_pickup.go_to_pose_goal(target_point, exe=False)
      if success and mode == 1:
        success = mantra_pickup.cartesian_move(end_state, target_point, cartesian_dis, scale=1.0)
      print success, time.time() - start
      exit(0)

    # ###################################   载入点云   ####################################
    if 1:
      reachable_points = np.loadtxt("reachable_points_mode%d.csv" % mode, delimiter=",")
      unreachable_points = np.loadtxt("unreachable_points_mode%d.csv" % mode, delimiter=",")
      xyz_ls = np.load("xyz_ls_mode%d.npy" % mode)

      analysis_z(xyz_ls, reachable_points, mode)
      # plot_points_3d(reachable_points, unreachable_points)
      exit(0)

    # #################################   创建目标点列表   ##################################
    step = 0.05 # mode0 -> (0.1-2min  0.05-15min)
    x_ls = np.arange(0.0, 0.7 + 1e-9, step)  # x坐标列表
    y_ls = np.arange(-0.6, 0.6 + 1e-9, step) # y坐标列表
    z_ls = np.arange(0, 0.55 + 1e-9, step) # z坐标列表
    # print len(x_ls), len(y_ls), len(z_ls)
    # print x_ls, y_ls, z_ls

    time_per_point = 0.2 if mode == 0 else 0.3  # 每个测试点耗时
    test_points_num = len(x_ls) * len(y_ls) * len(z_ls)
    print "test points num:", test_points_num, "need", test_points_num * time_per_point / 60, "min"
    # exit()

    reachable_points = []   # 可达点列表
    unreachable_points = [] # 不可达点列表
    reachable_num_z_ls = [] # 不同的z平面上可达点数目

    # #################################   判断是否可达   ##################################
    cnt = 0
    for z in z_ls:
      reachable_num_z = 0
      for x in x_ls:
        for y in y_ls:
          print "[%d/%d] try: [%.3f %.3f %.3f]" % (cnt, test_points_num, x, y, z)
          target_point = [x,y,z]
          
          if mode == 1: 
            plan_target_point = [x,y,z+cartesian_dis]  # 预留出直线轨迹长度
          else: plan_target_point = target_point

          success, end_state = mantra_pickup.go_to_pose_goal(plan_target_point, exe=False)
          if success and mode == 1:
            success = mantra_pickup.cartesian_move(end_state, plan_target_point, cartesian_dis, scale=1.0)
          
          if success:
            reachable_points.append(target_point)
            reachable_num_z += 1
          else:
            unreachable_points.append(target_point)

          cnt += 1
        
      reachable_num_z_ls.append([z, reachable_num_z])

    print "reachable_num_z_ls", reachable_num_z_ls

    # ##################################   保存数据   ####################################
    reachable_points = np.array(reachable_points)
    unreachable_points = np.array(unreachable_points)
    reachable_num_z_ls = np.array(reachable_num_z_ls)
    xyz_ls = np.array([x_ls, y_ls, z_ls])
    
    # 保存为csv
    np.savetxt("reachable_points_mode%d.csv" % mode, reachable_points, delimiter=",")
    np.savetxt("unreachable_points_mode%d.csv" % mode, unreachable_points, delimiter=",")
    np.savetxt("reachable_num_z_ls_mode%d.csv" % mode, reachable_num_z_ls, delimiter=",")
    np.save("xyz_ls_mode%d.npy" % mode, xyz_ls)

    # ###############################   数据处理及可视化   ##################################
    analysis_z(xyz_ls, reachable_points, mode)
    # plot_points_3d(reachable_points, unreachable_points)

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


