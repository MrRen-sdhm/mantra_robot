#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import geometry_msgs.msg
from transforms3d import quaternions
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'base_link'

class MoveItDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_pick_and_place_demo')
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.arm = arm
 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame(REFERENCE_FRAME)

    
    def obj_listener(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (obj_position, obj_orientation) = listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))
                rospy.loginfo("Camera pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", 
                    str(obj_position), str(obj_orientation))
                return obj_position, obj_orientation
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


if __name__ == "__main__":
    demo = MoveItDemo()
    rospy.sleep(5)
    while True:
        ############################## 获取目标物体位置 #########################
        s = raw_input("type in 's' to get camera pose:")
        if s is 's':
            print "get camera pose..."
            obj_position, obj_orientation = demo.obj_listener()

    
