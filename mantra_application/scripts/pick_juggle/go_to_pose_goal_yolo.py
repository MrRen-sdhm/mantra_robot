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
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
        arm.set_max_velocity_scaling_factor(0.8)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_planning_time(1) # 规划时间限制为2秒
        arm.set_num_planning_attempts(2) # 规划两次
        
        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        scene = moveit_commander.PlanningSceneInterface()

        print arm.get_current_pose(eef_link).pose

        sub = rospy.Subscriber('/detect_grasps_yolo/juggle_rects', Float64MultiArray, self.callback)
        self.juggle_rects = Float64MultiArray()

        self.box_name = ''
        self.scene = scene
        self.group = arm
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander

        self.move_distance = 0.1
        self.back_distance = 0.2

    def add_box(self, timeout=0.5):
      box_name = self.box_name
      scene = self.scene

      # 等待场景准备就绪
      rospy.sleep(0.5)

      # 设置场景物体的名称 
      table_id = 'table'  
      # 设置桌面的高度
      table_ground = 0
      # 设置table的三维尺寸[长, 宽, 高]
      table_size = [0.8, 0.99, 0.39]
      scene.remove_world_object(table_id)
      # 将个物体加入场景当中
      table_pose = geometry_msgs.msg.PoseStamped()
      table_pose.header.frame_id = 'base_link'
      table_pose.pose.position.x = 0.3
      table_pose.pose.position.y = 0.0
      table_pose.pose.position.z = table_ground - table_size[2] / 2.0 - 0.01
      table_pose.pose.orientation.w = 1.0
      scene.add_box(table_id, table_pose, table_size)

      # 设置场景物体的名称 
      left_wall_id = 'left_wall'  
      # 设置left_wall的三维尺寸[长, 宽, 高]
      left_wall_size = [0.8, 0.01, 1.0]
      scene.remove_world_object(left_wall_id)
      # 将个物体加入场景当中
      left_wall_pose = geometry_msgs.msg.PoseStamped()
      left_wall_pose.header.frame_id = 'base_link'
      left_wall_pose.pose.position.x = 0.4
      left_wall_pose.pose.position.y = -0.6
      left_wall_pose.pose.position.z = 0.3
      left_wall_pose.pose.orientation.w = 1.0
      # scene.add_box(left_wall_id, left_wall_pose, left_wall_size)

      # 设置场景物体的名称 
      bottom_wall_id = 'bottom_wall'  
      # 设置left_wall的三维尺寸[长, 宽, 高]
      bottom_wall_size = [1.0, 1.0, 0.01]
      scene.remove_world_object(bottom_wall_id)
      # 将个物体加入场景当中
      bottom_wall_pose = geometry_msgs.msg.PoseStamped()
      bottom_wall_pose.header.frame_id = 'base_link'
      bottom_wall_pose.pose.position.x = 0.5
      bottom_wall_pose.pose.position.y = 0.0
      bottom_wall_pose.pose.position.z = -0.39
      bottom_wall_pose.pose.orientation.w = 1.0
      scene.add_box(bottom_wall_id, bottom_wall_pose, bottom_wall_size)

    def go_to_pose_goal(self):
      group = self.group

      # pose_goal = geometry_msgs.msg.PoseStamped()
      # pose_goal.header.frame_id = self.reference_frame
      # pose_goal.header.stamp = rospy.Time.now()

      # pose_goal.pose.position.x = 0.436639379573
      # pose_goal.pose.position.y = 0.0203186032519
      # pose_goal.pose.position.z = -0.244789596306

      # pose_goal.pose.orientation.x = 0.706261175087
      # pose_goal.pose.orientation.y = 0.707490503556
      # pose_goal.pose.orientation.z = 0.0178783311735
      # pose_goal.pose.orientation.w = 0.0182402087981

      listener = tf.TransformListener()
      while not rospy.is_shutdown():
        try:
            (obj_position, obj_orientation) = listener.lookupTransform('/base_link', '/juggle', rospy.Time(0))
            rospy.loginfo("Selected grasp pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", 
              str(obj_position), str(obj_orientation))
            print("\n\nTry to plan a path to pick up the object once...")

            ## We can plan a motion for this group to a desired pose for the end-effector:
            pose_goal = geometry_msgs.msg.PoseStamped()
            pose_goal.header.frame_id = self.reference_frame
            pose_goal.header.stamp = rospy.Time.now()

            pose_goal.pose.position.x = obj_position[0]
            pose_goal.pose.position.y = obj_position[1]
            pose_goal.pose.position.z = obj_position[2] + 0.1

            euler = [-pi, 0, self.juggle_rects.data[3] * pi/180.0]
            print "[INFO] Angle:", self.juggle_rects.data[3]
            print "[INFO] Euler:", euler
            q = quaternion_from_euler(euler[0], euler[1], euler[2])
            pose_goal.pose.orientation.x = q[0]
            pose_goal.pose.orientation.y = q[1]
            pose_goal.pose.orientation.z = q[2]
            pose_goal.pose.orientation.w = q[3]

            print "[INFO] Pose:\n", pose_goal.pose.position

            middle_pose = cal_end_pose_by_quat(pose_goal.pose, -self.move_distance, 2)

            group.set_start_state_to_current_state()
            group.set_pose_target(middle_pose, self.eef_link)

            plan = group.go(wait=True)

            if(plan): # 规划完成，退出
              break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      group.stop()
      group.clear_pose_targets()

      # return plan

    def cartesian_move(self):
      cartesian = False
      arm = self.group
      # 获取当前位姿数据最为机械臂运动的起始位姿
      start_pose = arm.get_current_pose(self.eef_link).pose

      # 初始化路点列表
      waypoints = []
              
      # 将初始位姿加入路点列表
      waypoints.append(start_pose)

      # 按末端坐标系方向向量平移, 计算终点位姿
      wpose = cal_end_pose_by_quat(start_pose, self.move_distance, 2)

      if cartesian:
        waypoints.append(deepcopy(wpose))
      else:
        arm.set_pose_target(wpose)
        arm.go()

      if cartesian:
        ## 开始笛卡尔空间轨迹规划
        fraction = 0.0   #路径规划覆盖率
        maxtries = 10    #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
    
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                        
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
      
    def shift_pose_target(self):
      arm = self.group
      arm.shift_pose_target(2, self.back_distance, self.eef_link)
      arm.go()


    def callback(self, msg):
      rects = msg.data
      if len(rects) > 0:
        print "[x y z] [", rects[0], rects[1], rects[2], "]"
        self.juggle_rects.data = rects


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    mantra_pickup.add_box()

    mantra_pickup.group.set_named_target('pick_4')
    mantra_pickup.group.go()

    for i in range(4):
      print("Move to top of the object...")
      mantra_pickup.go_to_pose_goal()
      rospy.sleep(1)

      if True:
        print("Move approach to the object...")
        mantra_pickup.cartesian_move()
        rospy.sleep(1)
        print("Bring up the object...")
        mantra_pickup.shift_pose_target()
        rospy.sleep(1)

      print("Move to the init pose...")
      mantra_pickup.group.set_named_target('pick_4')
      mantra_pickup.group.go()
      rospy.sleep(2)

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


