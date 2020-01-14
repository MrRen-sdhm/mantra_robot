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

from mantra_application.srv import *
# from mantra_application.srv import MoveToPoseNamed, MoveToPoseNamedResponse
# from mantra_application.srv import MoveToPoseShift, MoveToPoseShiftResponse
# from mantra_application.srv import MoveToJointStates, MoveToJointStatesResponse
# from mantra_application.srv import GetCurrentPose, GetCurrentPoseResponse
# from mantra_application.srv import GetBaseEELink, GetBaseEELinkResponse
# from mantra_application.srv import SetVelScaling, SetVelScalingResponse


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
        plan = group.go()
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


def main():
    time.sleep(8)  # sleep to wait for moveit come up
    move_group = MoveGroup()
    rospy.spin()


if __name__ == '__main__':
    main()
