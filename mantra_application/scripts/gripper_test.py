#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander

GRIPPER_OPEN = [0]
GRIPPER_CLOSED = [0.04]

class MoveItPickAndPlaceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('gripper_test')
        
        # 创建一个发布抓取姿态的发布者
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        self.arm = arm
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander('gripper')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        self.end_effector_link = end_effector_link

        # 控制机械臂先回到初始化位置
        arm.set_named_target('test_2')
        arm.go()

        # 控制夹爪张开闭合
        for i in range(1):
            gripper.set_joint_value_target(GRIPPER_CLOSED)
            gripper.go()
            rospy.sleep(4)
            gripper.set_joint_value_target(GRIPPER_OPEN)
            gripper.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
