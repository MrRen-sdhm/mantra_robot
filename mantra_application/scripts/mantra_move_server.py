#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import time
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from math import pi
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
from std_msgs.msg import String
from transforms3d import quaternions

from mantra_application.srv import *
# from mantra_application.srv import MoveToPoseNamed, MoveToPoseNamedResponse
# from mantra_application.srv import MoveToPoseShift, MoveToPoseShiftResponse
# from mantra_application.srv import MoveToJointStates, MoveToJointStatesResponse
# from mantra_application.srv import GetCurrentPose, GetCurrentPoseResponse
# from mantra_application.srv import GetBaseEELink, GetBaseEELinkResponse
# from mantra_application.srv import SetVelScaling, SetVelScalingResponse


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


class MoveGroup(object):
    """MoveGroup"""

    def __init__(self):
        super(MoveGroup, self).__init__()

        rospy.init_node('mantra_move_server')

        print("[SRVICE] Mantra move server init done.")

        group_name = "arm"
        # group_name = "manipulator"
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander(group_name)

        # Get joint bounds
        joint_names = robot.get_joint_names(group=group_name)
        joint_bounds = []
        for joint_name in joint_names:
            joint = robot.get_joint(joint_name)
            joint_bounds.append(joint.bounds())
            print("[INFO] " + joint_name + "_bounds:", joint.bounds())

        group.allow_replanning(False)
        group.set_planning_time(0.5)
        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()

        self.robot = robot
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.joint_bounds = joint_bounds

        self.vel_scale = 0.5
        self.acc_scale = 0.25

        group.set_max_velocity_scaling_factor(self.vel_scale)
        group.set_max_acceleration_scaling_factor(self.acc_scale)

        rospy.Service('move_to_pose_named', MoveToPoseNamed, self.handle_move_to_pose_named)
        rospy.Service('move_to_poses_named', MoveToPosesNamed, self.handle_move_to_poses_named)
        rospy.Service('move_to_pose_shift', MoveToPoseShift, self.handle_move_to_pose_shift)
        rospy.Service('move_to_joint_states', MoveToJointStates, self.handle_move_to_joint_states)
        rospy.Service('move_ee_z', MoveEEZ, self.handle_move_ee_z)
        rospy.Service('get_current_pose', GetCurrentPose, self.handle_get_current_pose)
        rospy.Service('get_base_ee_link', GetBaseEELink, self.handle_get_base_ee_link)
        rospy.Service('set_vel_scaling', SetVelScaling, self.handle_set_vel_scaling)
        

    def handle_move_to_pose_named(self, req):
        ret = self.go_to_pose_named(req.pose_name)
        print("[SRVICE] Go to pose named: %s result:%s" % (str(req.pose_name), "Succeed" if ret else "Failed"))
        return MoveToPoseNamedResponse(ret)

    def handle_move_to_poses_named(self, req):
        ret = self.go_to_poses_named_continue(req.pose_names)
        print("[SRVICE] Go to poses named:", req.pose_names, end='')
        print(" result:%s" % "Succeed" if ret else "Failed")
        return MoveToPosesNamedResponse(ret)

    def handle_move_to_pose_shift(self, req):
        ret = self.go_to_pose_shift(req.axis, req.value)
        print("[SRVICE] Go to pose shift, axis:%d value:%.3f result:%s" % (req.axis, req.value, "Succeed" if ret else "Failed"))
        return MoveToPoseShiftResponse(ret)

    def handle_move_to_joint_states(self, req):
        ret = self.go_to_joint_state(req.joint_states)
        print("[SRVICE] Go to joint states result:%s" % "Succeed" if ret else "Failed")
        return MoveToJointStatesResponse(ret)

    def handle_move_ee_z(self, req):
        ret = self.move_ee_z(req.dis, req.scale)
        print("[SRVICE] Move ee z result:%s" % "Succeed" if ret else "Failed")
        return MoveEEZResponse(ret)

    def handle_get_current_pose(self, req):
        pose = self.get_current_pose()
        print("[SRVICE] Get current pose")
        return GetCurrentPoseResponse(pose)

    def handle_get_base_ee_link(self, req):
        print("[SRVICE] Get base and ee link")
        ee_link = self.eef_link
        base_link = self.planning_frame
        return GetBaseEELinkResponse(base_link, ee_link)

    def handle_set_vel_scaling(self, req):
        self.set_vel_scaling(req.scale)
        print("[SRVICE] Set velocity scaling:", req.scale)
        return SetVelScalingResponse(True)

    def go_to_joint_state(self, goal_positions):
        group = self.group

        # Check joint bounds
        goal_positions = list(goal_positions)
        for i in range(len(goal_positions)):
            if goal_positions[i] >= self.joint_bounds[i][1]:
                goal_positions[i] = self.joint_bounds[i][1]
            if goal_positions[i] <= self.joint_bounds[i][0]:
                goal_positions[i] = self.joint_bounds[i][0]

        # Print info
        print("[INFO] Go to joint state [", end=' ')
        for pos in goal_positions:
            print("%.3f" % pos, end=' ')
        print("]rad [", end=' ')
        for pos in goal_positions:
            print("%.3f" % (pos / pi * 180.0), end=' ')
        print("]deg")

        group.set_start_state_to_current_state()

        # Planning to a Joint Goal
        try:
            plan = group.go(goal_positions, wait=True)
        except:
            print("[WARN] target joints state not within bounds!")
            return False
        group.stop()
        group.clear_pose_targets()

        return plan

    def go_to_pose_named(self, pose_name):
        group = self.group
        group.set_start_state_to_current_state()
        group.set_named_target(pose_name)
        plan = group.go(wait=True) # 这里设置成阻塞的
        return plan

    def go_to_pose_shift(self, axis, value):
        group = self.group
        group.set_start_state_to_current_state()
        group.shift_pose_target(axis, value)
        plan = group.go()
        return plan

    def go_to_pose_goal(self):
        group = self.group

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.header.stamp = rospy.Time.now()

        pose_goal.pose.position.x = 0.500
        pose_goal.pose.position.y = 0.000
        pose_goal.pose.position.z = 0.500

        euler = [0, 0, 0]
        # euler = [0, -pi, 0]
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.eef_link)

        plan = group.go(wait=True)
        group.stop()

        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()
        return plan

    def go_to_poses_named_continue(self, names):
        group = self.group

        positions = []
        for name in names:
            position_dict = group.get_named_target_values(name)
            joint_names = sorted(position_dict.keys())
            position = []
            for joint_name in joint_names:
                position.append(position_dict[joint_name])
            positions.append(position)

        plan = self.stitch_positions(positions, scale=self.vel_scale)  # 使用全局速度比例, 通过set_vel_scaling设置
        result = group.execute(plan)

        return result

    # 重新计算轨迹时间
    def retime_trajectory(self, plan, scale):
        group = self.group

        ref_state = self.robot.get_current_state()
        retimed_plan = group.retime_trajectory(ref_state, plan, velocity_scaling_factor=scale)
        return retimed_plan

    # 拼接轨迹点
    def stitch_positions(self, positions, scale):
        group = self.group

        plan_list = []
        state = self.robot.get_current_state()

        # 路径规划, 连接各路径点
        for i in range(len(positions)):
            # 设置前一路径点为待规划的路径起点, 起始点除外
            if i > 0:
                state.joint_state.position = positions[i - 1]
            group.set_start_state(state)

            # 设置目标状态
            group.set_joint_value_target(positions[i])
            plan = group.plan()
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
        new_traj = self.retime_trajectory(new_traj, scale=scale)

        return new_traj

    def get_current_pose(self):
        group = self.group
        xyz = group.get_current_pose(self.eef_link).pose.position
        rpy = group.get_current_rpy(self.eef_link)
        pose = [xyz.x, xyz.y, xyz.z] + rpy
        print("[INFO] Get current pose:[%.2f %.2f %.2f %.2f %.2f %.2f]" % (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
        return pose

    def set_vel_scaling(self, scale):
        group = self.group

        self.vel_scale = scale  # 更新全局速度比例
        group.set_max_velocity_scaling_factor(scale)


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

    def cartesian_move(self, dis, scale):
      group = self.group
      # 前进
      waypoints = []
      wpose = group.get_current_pose().pose
      waypoints.append(deepcopy(wpose))

      wpose = cal_end_pose_by_quat(wpose, dis, 2)
      waypoints.append(deepcopy(wpose))

      ret = self.plan_cartesian(waypoints, scale=scale)
      return ret
    
    def move_ee_z(self, dis, scale):
        group = self.group
        ret = self.cartesian_move(dis, scale)
        return ret


def main():
    print("Mantra move server init...")
    time.sleep(8)  # sleep to wait for moveit come up
    print("Mantra move server started!")
    move_group = MoveGroup()
    rospy.spin()


if __name__ == '__main__':
    main()
