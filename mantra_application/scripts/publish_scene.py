#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, time

import moveit_commander
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import PlanningScene, ObjectColor


class MantraPickup:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        robot = moveit_commander.RobotCommander()
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
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


if __name__ == "__main__":
  try:
    mantra_pickup = MantraPickup()

    mantra_pickup.add_box()

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass


