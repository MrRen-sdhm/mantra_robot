#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
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


class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        # arm.set_goal_position_tolerance(0.01)
        # arm.set_goal_orientation_tolerance(0.1)

        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂运动到之前设置的姿态
        # arm.set_named_target('cali_2')
        # arm.set_named_target('test_1')
        arm.set_joint_value_target([0.055, 0.317, -0.033, -1.292, -0.099, -0.087, -1.575])
        arm.go()

        # exit()

        pose = cal_end_pose_by_quat(arm.get_current_pose().pose, -0.2, 2)
        arm.set_pose_target(pose)
        arm.go()

        # raw_input()
        
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose().pose

        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)

        # 按末端坐标系方向向量平移, 计算终点位姿
        wpose = cal_end_pose_by_quat(start_pose, 0.35, 2)
        waypoints.append(deepcopy(wpose))

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

        def move_cartesian(waypoints, scale):
            group = arm
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
                plan = scale_trajectory_speed(plan, scale)
                group.execute(plan)
                rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
                    maxtries) + " attempts.")

        move_cartesian(waypoints, 0.5)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
