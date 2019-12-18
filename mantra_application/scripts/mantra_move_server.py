#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import time
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from mantra_application.srv import MoveToPoseNamed, MoveToPoseNamedResponse
from mantra_application.srv import MoveToPoseShift, MoveToPoseShiftResponse
from mantra_application.srv import MoveToJointStates, MoveToJointStatesResponse
from mantra_application.srv import GetCurrentPose, GetCurrentPoseResponse
from mantra_application.srv import GetBaseEELink, GetBaseEELinkResponse
from mantra_application.srv import SetVelScaling, SetVelScalingResponse


class MoveGroup(object):
    """MoveGroup"""

    def __init__(self):
        super(MoveGroup, self).__init__()

        rospy.init_node('mantra_move_server')
        rospy.Service('move_to_pose_named', MoveToPoseNamed, self.handle_move_to_pose_named)
        rospy.Service('move_to_pose_shift', MoveToPoseShift, self.handle_move_to_pose_shift)
        rospy.Service('move_to_joint_states', MoveToJointStates, self.handle_move_to_joint_states)
        rospy.Service('get_current_pose', GetCurrentPose, self.handle_get_current_pose)
        rospy.Service('get_base_ee_link', GetBaseEELink, self.handle_get_base_ee_link)
        rospy.Service('set_vel_scaling', SetVelScaling, self.handle_set_vel_scaling)

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
        # group.set_goal_position_tolerance(0.01)
        # group.set_goal_orientation_tolerance(0.05)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()

        self.robot = robot
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.joint_bounds = joint_bounds

    def handle_move_to_pose_named(self, req):
        ret = self.go_to_pose_named(req.pose_name)
        print("[SRVICE] Go to pose named: %s result:%s" % (str(req.pose_name), "Succeed" if ret else "Failed"))
        return MoveToPoseNamedResponse(ret)

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
        group.set_named_target(pose_name)
        plan = group.go()
        return plan

    def go_to_pose_shift(self, axis, value):
        group = self.group
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

    def get_current_pose(self):
        group = self.group
        xyz = group.get_current_pose(self.eef_link).pose.position
        rpy = group.get_current_rpy(self.eef_link)
        pose = [xyz.x, xyz.y, xyz.z] + rpy
        print("[INFO] Get current pose:[%.2f %.2f %.2f %.2f %.2f %.2f]" % (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
        return pose

    def set_vel_scaling(self, scale):
        group = self.group
        group.set_max_velocity_scaling_factor(scale)


def main():
    time.sleep(5)  # sleep to wait for moveit come up
    move_group = MoveGroup()
    rospy.spin()


if __name__ == '__main__':
    main()
