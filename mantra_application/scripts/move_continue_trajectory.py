#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotState

from copy import deepcopy
from transforms3d import quaternions


class MoveItDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('move_continue_demo', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        robot = moveit_commander.RobotCommander()
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        # arm.set_goal_position_tolerance(0.01)
        # arm.set_goal_orientation_tolerance(0.1)

        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(0.5)

        arm.set_start_state_to_current_state()
        arm.set_named_target("fusion_left")
        arm.go()

        # 轨迹速度加速度缩放
        def scale_trajectory_vel_acc(traj, vel_scale, acc_scale):
            new_traj = RobotTrajectory()
            new_traj.joint_trajectory = traj.joint_trajectory

            n_joints = len(traj.joint_trajectory.joint_names)
            n_points = len(traj.joint_trajectory.points)
            points = list(traj.joint_trajectory.points)

            for i in range(n_points):
                point = JointTrajectoryPoint()
                point.positions = traj.joint_trajectory.points[i].positions

                point.time_from_start = traj.joint_trajectory.points[i].time_from_start / vel_scale
                point.velocities = list(traj.joint_trajectory.points[i].velocities)
                point.accelerations = list(traj.joint_trajectory.points[i].accelerations)

                for j in range(n_joints):
                    point.velocities[j] = point.velocities[j] * vel_scale
                    point.accelerations[j] = point.accelerations[j] * acc_scale * acc_scale
                points[i] = point

            new_traj.joint_trajectory.points = points
            return new_traj

        # 重新计算轨迹时间
        def retime_trajectory(plan, scale):
            ref_state = robot.get_current_state()
            retimed_plan = arm.retime_trajectory(ref_state, plan, velocity_scaling_factor=scale)
            return retimed_plan

        # 拼接轨迹点
        def stitch_positions(positions):
            plan_list = []
            state = robot.get_current_state()

            # 路径规划, 连接各路径点
            for i in range(len(positions)):
                # 设置前一路径点为待规划的路径起点, 起始点除外
                if i > 0:
                    state.joint_state.position = positions[i-1]
                arm.set_start_state(state)

                # 设置目标状态
                arm.set_joint_value_target(positions[i])
                plan = arm.plan()
                plan_list.append(plan)

            # 创建新轨迹, 重新计算时间
            new_traj = RobotTrajectory()
            new_traj.joint_trajectory.joint_names = plan_list[0].joint_trajectory.joint_names

            # 轨迹点拼接
            new_points = []
            for plan in plan_list:
                new_points += list(plan.joint_trajectory.points)
            new_traj.joint_trajectory.points = new_points

            # 重新计算轨迹时间
            new_traj = retime_trajectory(new_traj, scale=0.5)

            return new_traj

        # positions = [[0.055, 0.317, -0.033, -1.292, -0.099, -0.087, -1.575],
        #              [1.055, 1.317, -1.033, -1.292, -0.099, -0.087, -1.575],
        #              [1.055, -1.317, 1.033, 1.292, -0.099, 1.087, -1.575]]
        #
        # plan = stitch_positions(positions)
        # arm.execute(plan)

        positions_name = ['cali_3', 'fusion_left2', 'fusion_left1', 'fusion_1', 'fusion_right1', 'fusion_right2']
        positions = []
        for name in positions_name:
            position_dict = arm.get_named_target_values(name)
            joint_names = sorted(position_dict.keys())
            position = []
            for joint_name in joint_names:
                position.append(position_dict[joint_name])
            positions.append(position)

        plan = stitch_positions(positions)

        # 速度加速度缩放
        # plan = scale_trajectory_vel_acc(plan, 0.25, 0.25)
        # new_traj = retime_trajectory(plan, scale=0.25)

        arm.execute(plan)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        exit()

        # arm.get_named_target_values()

        # ###########################################  test  #############################################
        # 轨迹列表
        plan_list = []

        # 设置初始状态
        state = robot.get_current_state()
        arm.set_start_state(state)
        # 设置目标状态
        aim_position1 = [0.055, 0.317, -0.033, -1.292, -0.099, -0.087, -1.575]
        arm.set_joint_value_target(aim_position1)
        plan1 = arm.plan()
        plan_list.append(plan1)

        # 设置初始状态
        state.joint_state.position = aim_position1
        arm.set_start_state(state)
        # 设置目标状态
        aim_position2 = [1.055, 1.317, -1.033, -1.292, -0.099, -0.087, -1.575]
        arm.set_joint_value_target(aim_position2)
        plan2 = arm.plan()
        plan_list.append(plan2)

        # 设置初始状态
        state.joint_state.position = aim_position2
        arm.set_start_state(state)
        # 设置目标状态
        aim_position3 = [1.055, -1.317, 1.033, 1.292, -0.099, 1.087, -1.575]
        arm.set_joint_value_target(aim_position3)
        plan3 = arm.plan()
        plan_list.append(plan3)

        new_traj = RobotTrajectory()
        new_traj.joint_trajectory.joint_names = plan1.joint_trajectory.joint_names

        # 轨迹点拼接
        new_points = []
        for plan in plan_list:
            new_points += list(plan.joint_trajectory.points)
        new_traj.joint_trajectory.points = new_points

        # 重新计算轨迹时间
        new_traj = retime_trajectory(new_traj, scale=0.5)

        # 执行轨迹
        arm.execute(new_traj)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
