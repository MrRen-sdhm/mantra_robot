#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 到达物体上方与前进动作先规划后执行，前一动作执行成功再执行后一动作

import rospy, sys, tf, time
import thread

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
from math import pi
from std_msgs.msg import String
from tf.transformations import *
from transforms3d import quaternions

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from yinshi_driver.srv import *
from gpd_ros.srv import detect_grasps


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
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        robot = moveit_commander.RobotCommander()
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(False)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)

        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(0.5)

        arm.set_planning_time(0.08) # 规划时间限制为2秒
        # arm.set_num_planning_attempts(1) # 规划1次
        
        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        scene = moveit_commander.PlanningSceneInterface()
        scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        print "[INFO] Current pose:\n", arm.get_current_pose().pose

        self.scene = scene
        self.scene_pub = scene_pub
        self.colors = dict()

        self.group = arm
        self.robot = robot
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander

        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.gripper_len = 0.095  # 手爪实际长度0.165m, 虚拟夹爪深度0.075m 0.16-0.065=0.095m
        self.approach_distance = 0.06
        self.back_distance = 0.05

        # sub and pub point cloud
        self.point_cloud = None
        self.update_cloud_flag = False
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback_pointcloud)
        thread.start_new_thread(self.publish_pointcloud, ())

    def callback_pointcloud(self, data):
      assert isinstance(data, PointCloud2)
      if self.update_cloud_flag:
          self.update_cloud_flag = False
          self.point_cloud = data
    
    def publish_pointcloud(self):
      pub = rospy.Publisher('grasp_point_cloud', PointCloud2, queue_size=10)
      rate = rospy.Rate(5)  # 10hz
      while not rospy.is_shutdown():
        if isinstance(self.point_cloud, PointCloud2):
          pub.publish(self.point_cloud)
        rate.sleep()

    def update_cloud(self):
      # update point cloud
      time.sleep(0.5)
      self.update_cloud_flag = True

    def setColor(self, name, r, g, b, a = 0.9):
      """ 设置场景物体的颜色 """
      # 初始化moveit颜色对象
      color = ObjectColor()
      
      # 设置颜色值
      color.id = name
      color.color.r = r
      color.color.g = g
      color.color.b = b
      color.color.a = a
      
      # 更新颜色字典
      self.colors[name] = color

    def sendColors(self):
      """ 将颜色设置发送并应用到moveit场景当中 """
      # 初始化规划场景对象
      p = PlanningScene()

      # 需要设置规划场景是否有差异     
      p.is_diff = True
      
      # 从颜色字典中取出颜色设置
      for color in self.colors.values():
          p.object_colors.append(color)
      
      # 发布场景物体颜色设置
      self.scene_pub.publish(p)

    def add_box(self, timeout=1.0):
      scene = self.scene

      # 等待场景准备就绪
      rospy.sleep(0.5)

      table_id = 'table'  
      # 设置桌面的高度
      table_ground = 0
      # 设置table的三维尺寸[长, 宽, 高]
      table_size = [0.4, 0.4, 0.2]
      scene.remove_world_object(table_id)
      # 将个物体加入场景当中
      table_pose = geometry_msgs.msg.PoseStamped()
      table_pose.header.frame_id = 'base_link'
      table_pose.pose.position.x = 0
      table_pose.pose.position.y = 0.0
      table_pose.pose.position.z = table_ground - table_size[2] / 2.0 - 0.01
      table_pose.pose.orientation.w = 1.0
      scene.add_box(table_id, table_pose, table_size)
      self.setColor(table_id, 0.0, 0.0, 1.0, 0.9)

      right_wall_id = 'right_wall'  
      # 设置right_wall的三维尺寸[长, 宽, 高]
      right_wall_size = [0.6, 0.01, 0.6]
      scene.remove_world_object(right_wall_id)
      # 将个物体加入场景当中
      right_wall_pose = geometry_msgs.msg.PoseStamped()
      right_wall_pose.header.frame_id = 'base_link'
      right_wall_pose.pose.position.x = 0.4
      right_wall_pose.pose.position.y = -0.6
      right_wall_pose.pose.position.z = 0.3
      right_wall_pose.pose.orientation.w = 1.0
      scene.add_box(right_wall_id, right_wall_pose, right_wall_size)
      self.setColor(right_wall_id, 1.0, 1.0, 1.0, 0.05)

      left_wall_id = 'left_wall'  
      # 设置left_wall的三维尺寸[长, 宽, 高]
      left_wall_size = [0.6, 0.01, 0.6]
      scene.remove_world_object(left_wall_id)
      # 将个物体加入场景当中
      left_wall_pose = geometry_msgs.msg.PoseStamped()
      left_wall_pose.header.frame_id = 'base_link'
      left_wall_pose.pose.position.x = 0.4
      left_wall_pose.pose.position.y = 0.55
      left_wall_pose.pose.position.z = 0.3
      left_wall_pose.pose.orientation.w = 1.0
      scene.add_box(left_wall_id, left_wall_pose, left_wall_size)
      self.setColor(left_wall_id, 1.0, 1.0, 1.0, 0.05)

      back_wall_id = 'back_wall'  
      # 设置left_wall的三维尺寸[长, 宽, 高]
      back_wall_size = [0.01, 0.8, 1.0]  # 0.33
      scene.remove_world_object(back_wall_id)
      # 将个物体加入场景当中
      back_wall_pose = geometry_msgs.msg.PoseStamped()
      back_wall_pose.header.frame_id = 'base_link'
      back_wall_pose.pose.position.x = -0.3
      back_wall_pose.pose.position.y = 0.0
      back_wall_pose.pose.position.z = back_wall_size[2]/2
      back_wall_pose.pose.orientation.w = 1.0
      scene.add_box(back_wall_id, back_wall_pose, back_wall_size)
      self.setColor(back_wall_id, 1.0, 1.0, 1.0, 0.05)

      bottom_wall_id = 'bottom_wall'  
      # 设置left_wall的三维尺寸[长, 宽, 高]
      bottom_wall_size = [0.5, 0.55, 0.25]  # 0.33
      scene.remove_world_object(bottom_wall_id)
      # 将个物体加入场景当中
      bottom_wall_pose = geometry_msgs.msg.PoseStamped()
      bottom_wall_pose.header.frame_id = 'base_link'
      bottom_wall_pose.pose.position.x = 0.55
      bottom_wall_pose.pose.position.y = 0.0
      bottom_wall_pose.pose.position.z = bottom_wall_size[2]/2
      bottom_wall_pose.pose.orientation.w = 1.0
      scene.add_box(bottom_wall_id, bottom_wall_pose, bottom_wall_size)
      self.setColor(bottom_wall_id, 0.0, 1.0, 0.0, 0.4)

      # 将场景中的颜色设置发布
      self.sendColors()
      time.sleep(timeout)

      # exit()

    def get_camera_pose(self):
      while not rospy.is_shutdown():
        try:
          (cam_trans, cam_quat) = self.listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))
          return (cam_trans, cam_quat)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          pass

    def hand_control_client(self, command, pose=None):
        if pose is None:
            pose = -1
        else:
            pose = int(pose)

        rospy.wait_for_service('hand_control')
        try:
            hand_control = rospy.ServiceProxy('hand_control', HandControl)
            resp = hand_control(command, pose)
            return resp.success
        except rospy.ServiceException, e:
            print "[SRVICE] Service call failed: %s" % e

    def get_grasps_client(self):
        rospy.wait_for_service('/gpd_server/detect_grasps', timeout=1)
        try:
            detectGrasps = rospy.ServiceProxy('/gpd_server/detect_grasps', detect_grasps)
            resp = detectGrasps()
            print "[SRVICE] Get %d grasps" % len(resp.grasp_configs.grasps)
            return resp.grasp_configs.grasps
        except rospy.ServiceException, e:
            print "[SRVICE] Did't get any grasps."

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

    def find_better_traj(self, pose_goal):
      group = self.group

      # get short traj
      traj_ls = []
      traj_len_ls = []
      pose_goal_ls = []
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
        pose_goal_ls.append(pose_goal)

      better_traj_idx = traj_len_ls.index(min(traj_len_ls))
      better_traj = traj_ls[better_traj_idx]
      better_traj_len = traj_len_ls[better_traj_idx]
      better_pose_goal = pose_goal_ls[better_traj_idx]

      print "[INFO] Better traj index:", better_traj_idx
      print "[INFO] Better traj len:", better_traj_len

      if better_traj_len == 0:
        print "[WARN] Not find better traj"

      return better_traj, better_traj_len, better_pose_goal

    def go_to_pose_goal(self):
      group = self.group

      listener = tf.TransformListener()
      while not rospy.is_shutdown():
        try:
            (obj_position, obj_orientation) = listener.lookupTransform('/base_link', '/grasp', rospy.Time(0))
            print("Try to plan a path to pick up the object once...")

            ## We can plan a motion for this group to a desired pose for the end-effector:
            pose_goal = geometry_msgs.msg.PoseStamped()
            pose_goal.header.frame_id = self.reference_frame
            pose_goal.header.stamp = rospy.Time.now()

            pose_goal.pose.position.x = obj_position[0]
            pose_goal.pose.position.y = obj_position[1]
            pose_goal.pose.position.z = obj_position[2]

            pose_goal.pose.orientation.x = obj_orientation[0]
            pose_goal.pose.orientation.y = obj_orientation[1]
            pose_goal.pose.orientation.z = obj_orientation[2]
            pose_goal.pose.orientation.w = obj_orientation[3]

            # 沿z轴退回
            pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -(self.approach_distance+self.gripper_len), 2)
            # pose_goal.pose = cal_end_pose_by_quat(pose_goal.pose, -(self.gripper_len), 2)

            group.set_start_state_to_current_state()
            group.set_pose_target(pose_goal, self.eef_link)

            print "[INFO] Try pose:", pose_goal.pose
            # plan = group.go(wait=True)

            # rotate 0 or 180 to find a better traj
            better_traj, better_traj_len = self.find_better_traj(pose_goal)
            if better_traj_len > 0 and better_traj_len < 50:
              plan = group.execute(better_traj, wait=True)

              # move success, break
              break

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      group.stop()
      group.clear_pose_targets()

      return plan

    def get_obj_pose(self, grasp):
        # (1)获取相机坐标系下物体位姿
        cam2obj_trans = [grasp.position.x, grasp.position.y, grasp.position.z]

        print "\033[1;32m%s\033[0m" % "[INFO] cam2obj_trans:\n", cam2obj_trans

        # 旋转矩阵的第一列为坐标系x轴方向向量，第二列为坐标系y轴方向向量，第三列为坐标系z轴方向向量
        cam2obj_rot = np.array([(grasp.approach.x, grasp.binormal.x, grasp.axis.x),
                                (grasp.approach.y, grasp.binormal.y, grasp.axis.y),
                                (grasp.approach.z, grasp.binormal.z, grasp.axis.z)])
        print "\033[1;32m%s\033[0m" % "[INFO] cam2obj_rot:\n", cam2obj_rot

        # (2)获取机器人坐标系下相机位姿
        base2cam_trans, base2cam_quat = self.get_camera_pose()
        # print "\033[1;32m%s\033[0m" % "[INFO] base2cam_trans:\n", base2cam_trans
        # print "\033[1;32m%s\033[0m" % "[INFO] base2cam_quat:\n", base2cam_quat

        def cal_obj_pose_in_world(base2cam_trans, base2cam_quat, cam2obj_trans, cam2obj_rot):
          # 旋转矩阵转四元数
          cam2obj_rot44 = numpy.identity(4)
          cam2obj_rot44[:3, :3] = cam2obj_rot
          cam2obj_quat = quaternion_from_matrix(cam2obj_rot44)

          # cam2obj转换为齐次变换矩阵
          cam2obj_trans = translation_matrix(cam2obj_trans)
          cam2obj_quat = quaternion_matrix(cam2obj_quat)
          cam2obj_mat44 = np.dot(cam2obj_trans, cam2obj_quat)

          '''世界->物体'''
          # base2cam转换为齐次变换矩阵
          base2cam_trans44 = translation_matrix(base2cam_trans)
          base2cam_quat44 = quaternion_matrix(base2cam_quat)
          base2cam_mat44 = np.dot(base2cam_trans44, base2cam_quat44)

          # base坐标系下物体位姿
          base2obj_mat44 = np.dot(base2cam_mat44, cam2obj_mat44)  # base->cam->obj
          base2obj_trans = translation_from_matrix(base2obj_mat44)
          base2obj_quat = quaternion_from_matrix(base2obj_mat44)

          return base2obj_trans, base2obj_quat

        # (3)计算机器人坐标系下物体位姿
        base2obj_trans, base2obj_quat = cal_obj_pose_in_world(base2cam_trans, base2cam_quat, cam2obj_trans, cam2obj_rot)
        
        base2obj_quat = quaternion_multiply(base2obj_quat, quaternion_about_axis(pi/2, [0, 1, 0]))  # 绕y轴旋转90度
        print "\033[1;32m%s\033[0m" % "[INFO] base2obj_trans:\n", base2obj_trans
        print "\033[1;32m%s\033[0m" % "[INFO] base2obj_quat:\n", base2obj_quat

        return base2obj_trans, base2obj_quat
        
    def go_to_pose_goal_service(self):
      group = self.group

      while not rospy.is_shutdown():
        grasps = self.get_grasps_client()
        if grasps is None:
          continue

        for grasp in grasps:
          if grasp.score.data < 0.9:  # score limit
            continue

          print "[INFO] Try a grasp, score:", grasp.score.data
          obj_trans, obj_quat = self.get_obj_pose(grasp)
          
          # 发布物体姿态
          self.broadcaster.sendTransform(obj_trans, obj_quat, rospy.Time.now(), "obj_pose", self.reference_frame)

          print("[INFO] Try to plan a path to pick up the object once...")
          pose_goal = geometry_msgs.msg.PoseStamped()
          pose_goal.header.frame_id = self.reference_frame
          pose_goal.header.stamp = rospy.Time.now()

          pose_goal.pose.position.x = obj_trans[0]
          pose_goal.pose.position.y = obj_trans[1]
          pose_goal.pose.position.z = obj_trans[2]

          pose_goal.pose.orientation.x = obj_quat[0]
          pose_goal.pose.orientation.y = obj_quat[1]
          pose_goal.pose.orientation.z = obj_quat[2]
          pose_goal.pose.orientation.w = obj_quat[3]

          # 沿z轴退回
          pose_back = geometry_msgs.msg.PoseStamped()
          pose_back.header.frame_id = self.reference_frame
          pose_back.header.stamp = rospy.Time.now()
          pose_back.pose = cal_end_pose_by_quat(pose_goal.pose, -(self.approach_distance+self.gripper_len), 2)
          # pose_back.pose = cal_end_pose_by_quat(pose_goal.pose, -(self.gripper_len), 2)

          print "[INFO] Try pose:", pose_back.pose
          # plan = group.go(wait=True)

          # rotate 0 or 180 to find a better traj and get better pose_goal
          better_traj, better_traj_len, better_pose_back = self.find_better_traj(pose_back)

          # cartesian plan
          if better_traj_len > 0:
            end_state = better_traj.joint_trajectory.points[-1].positions  # get end_state of last plan
            waypoints = [better_pose_back.pose]
            # cal pose goal, can't use origin pose goal
            pose_goal.pose = cal_end_pose_by_quat(better_pose_back.pose, self.approach_distance, 2)
            waypoints.append(pose_goal.pose)
            cartesian_plan = self.plan_cartesian(waypoints, start_state=end_state, scale=0.5)

          if better_traj_len > 0 and better_traj_len < 50 and cartesian_plan is not None:
            print "\033[1;36m%s\033[0m" % "[INFO] Move to top of the object..."
            plan = group.execute(better_traj, wait=True)
            print "\033[1;36m%s\033[0m" % "[INFO] Move approach to the object..."
            plan = group.execute(cartesian_plan, wait=True)
            
            if plan:
              return True # move success, return
          elif better_traj_len >= 40:
            print "\033[1;32m%s\033[0m" % "[INFO] Find better traj but too complex!"

    def plan_cartesian(self, waypoints, scale=1.0, start_state=None):
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

    def cartesian_move(self, dir, scale):
      group = self.group
      # 前进
      waypoints = []
      wpose = group.get_current_pose().pose
      waypoints.append(deepcopy(wpose))

      wpose = cal_end_pose_by_quat(wpose, dir * self.approach_distance, 2)
      waypoints.append(deepcopy(wpose))

      self.plan_cartesian(waypoints, scale=scale) 
      
    def shift_pose_target(self):
      arm = self.group
      arm.shift_pose_target(2, self.back_distance, self.eef_link)
      arm.go()

      # arm.shift_pose_target(1, 0.1, self.eef_link)
      # arm.go()

    def go_to_pose_manual(self):
      group = self.group

      # group.set_joint_value_target([-0.9992, -0.3808, -2.0633, 0.9237, -0.2045, 1.8237, -1.5502])
      # group.go()

      # trans = [0.403627486802, 0.100759495816, 0.562776646167]
      # quat = [-0.729093757762, 0.67408609808, -0.117612364822, 0.0140554761174]

      trans = [0.3799461742419498, -0.07236909519290599, 0.5284943296666942]
      quat = [-0.934448916276, 0.328090497929, -0.0265391078696, 0.135857853704]

      pose_goal = geometry_msgs.msg.PoseStamped()
      pose_goal.header.frame_id = self.reference_frame
      pose_goal.header.stamp = rospy.Time.now()

      pose_goal.pose.position.x = trans[0]
      pose_goal.pose.position.y = trans[1]
      pose_goal.pose.position.z = trans[2]

      # euler = [-0.296, -0.153, -1.507]
      # q = quaternion_from_euler(euler[0], euler[1], euler[2])
      # pose_goal.pose.orientation.x = q[0]
      # pose_goal.pose.orientation.y = q[1]
      # pose_goal.pose.orientation.z = q[2]
      # pose_goal.pose.orientation.w = q[3]

      pose_goal.pose.orientation.x = quat[0]
      pose_goal.pose.orientation.y = quat[1]
      pose_goal.pose.orientation.z = quat[2]
      pose_goal.pose.orientation.w = quat[3]

      group.set_pose_target(pose_goal)
      # group.go()

      # 初始化路点列表
      waypoints = []

      wpose = group.get_current_pose().pose
      print group.get_current_pose()
      waypoints.append(deepcopy(wpose))

      wpose = cal_end_pose_by_quat(wpose, 0.01, 2)
      print wpose
      waypoints.append(deepcopy(wpose))

      self.move_cartesian(waypoints, 0.20)

      #################################
      # # exit()
      # waypoints = []

      # wpose = group.get_current_pose().pose
      # waypoints.append(deepcopy(wpose))

      # wpose = cal_end_pose_by_quat(wpose, -0.15, 2)
      # waypoints.append(deepcopy(wpose))

      # # wpose = cal_end_pose_by_quat(pose_goal.pose, -0.09, 2)
      # # waypoints.append(deepcopy(wpose))

      # self.move_cartesian(waypoints, 0.10)


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    mantra_pickup.add_box()

    mantra_pickup.group.set_named_target('pick_1')
    mantra_pickup.group.go()
    mantra_pickup.update_cloud()

    mantra_pickup.hand_control_client("Open")

    # mantra_pickup.go_to_pose_manual()

    for i in range(12):
      print "\033[1;36m%s\033[0m" % "[INFO] Plan a traj connect Top and Approach..."
      # mantra_pickup.go_to_pose_goal()
      mantra_pickup.go_to_pose_goal_service()

      # print "\033[1;36m%s\033[0m" % "[INFO] Move approach to the object..."
      # mantra_pickup.cartesian_move(dir=1, scale=0.3)

      print "\033[1;36m%s\033[0m" % "[INFO] Close the gripper..."
      mantra_pickup.hand_control_client("Close")
      time.sleep(0.5)

      print "\033[1;36m%s\033[0m" % "[INFO] Bring up the object..."
      # mantra_pickup.shift_pose_target()
      mantra_pickup.cartesian_move(dir=-1, scale=0.5)
      mantra_pickup.group.set_named_target('middle_right_1')
      mantra_pickup.group.go()

      mantra_pickup.group.set_named_target('place_right_1')
      mantra_pickup.group.go()

      print "\033[1;36m%s\033[0m" % "[INFO] Open the gripper..."
      mantra_pickup.hand_control_client("Open")
      time.sleep(0.5)

      mantra_pickup.group.set_named_target('pick_1')
      mantra_pickup.group.go()
      mantra_pickup.update_cloud()

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


