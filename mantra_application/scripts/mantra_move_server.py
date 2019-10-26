#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from mantra_application.srv import MoveToPoseNamed, MoveToPoseNamedResponse


class MoveGroup(object):
    """MoveGroup"""

    def __init__(self):
        super(MoveGroup, self).__init__()

        rospy.init_node('mantra_move_server')
        srv = rospy.Service('move_to_pose_named', MoveToPoseNamed, self.handle_move_to_pose_named)
        print "[SRVICE] Mantra move server init done."

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander("arm")

        group.allow_replanning(True)
        group.set_goal_position_tolerance(0.01)
        group.set_goal_orientation_tolerance(0.05)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()

        self.robot = robot
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link

    def handle_move_to_pose_named(self, req):
        self.go_to_pose_named(req.pose_name)
        print "[SRVICE] Go to pose named: %s" % str(req.pose_name)
        return MoveToPoseNamedResponse(True)

    def go_to_joint_state(self):
        group = self.group

        # Planning to a Joint Goal
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 4
        joint_goal[2] = 0
        joint_goal[3] = -pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3
        joint_goal[6] = 0

        group.go(joint_goal, wait=True)
        group.stop()
        group.clear_pose_targets()

        return True

    def go_to_pose_named(self, pose_name):
        group = self.group

        group.set_named_target(pose_name)
        group.go()
        group.stop()
        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()
        return True

    def go_to_pose_goal(self):
        group = self.group

        # We can plan a motion for this group to a desired pose for the end-effector:

        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position.x = 0.0
        # pose_goal.position.y = 0.0
        # pose_goal.position.z = 0.0

        # euler = [0, 0, pi/4]
        # q = quaternion_from_euler(euler[0], euler[1], euler[2])
        # pose_goal.orientation.x = q[0]
        # pose_goal.orientation.y = q[1]
        # pose_goal.orientation.z = q[2]
        # pose_goal.orientation.w = q[3]
        # group.set_pose_target(pose_goal)

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


def handle_move_to_pose_named(req):
    print "[SRVICE] Go to pose named: %s" % str(req.pose_name)
    return MoveToPoseNamedResponse(True)


def main():
    move_group = MoveGroup()
    rospy.spin()


if __name__ == '__main__':
    main()
