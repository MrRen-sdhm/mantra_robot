#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, tf
import geometry_msgs.msg
import actionlib
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

GRIPPER_OPEN = [0]
GRIPPER_CLOSED = [0.04]

class MoveItDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('gripper_test')
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        arm.set_max_velocity_scaling_factor(0.3)
        arm.set_max_acceleration_scaling_factor(0.5)
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander('gripper')
        self.gripper = gripper

        # 控制机械臂先回到初始位置
        arm.set_named_target('home')
        arm.go()

        # 控制夹爪张开
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()

        rospy.sleep(0.5)

        # 控制机械臂到目标位置
        arm.set_named_target('test_2')
        arm.go()

        # 控制夹爪闭合
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()

        # 控制机械臂先回到初始位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # 使用actionlib控制夹抓, 实际movegroup,go使用的就是actionlib
    def gripperAction(self):
        gripper_action = actionlib.SimpleActionClient("gripper", GripperCommandAction)
        rospy.loginfo("gripper wait_for_server.")
        gripper_action.wait_for_server()
        rospy.loginfo("gripper action connected.")
        
        print "grip is planning"
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 1.0
        gripper_goal.command.position = float(0.04)

        gripper_action.send_goal(gripper_goal)
        rospy.loginfo("gripper done")


if __name__ == "__main__":
    moveit_demo = MoveItDemo()
    # moveit_demo.gripperAction()

    
