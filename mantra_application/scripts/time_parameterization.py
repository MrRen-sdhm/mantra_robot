#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_commander import RobotCommander


class PythonTimeParameterizationTest(unittest.TestCase):
    PLANNING_GROUP = "arm"

    @classmethod
    def setUpClass(self):
        self.commander = RobotCommander("robot_description")
        self.group = self.commander.get_group(self.PLANNING_GROUP)

    @classmethod
    def tearDown(self):
        pass

    def plan(self):
        start_pose = self.group.get_current_pose().pose
        goal_pose = self.group.get_current_pose().pose
        goal_pose.position.z -= 0.1
        (plan, fraction) = self.group.compute_cartesian_path([start_pose, goal_pose], 0.005, 0.0)
        self.assertEqual(fraction, 1.0, "Cartesian path plan failed")
        return plan

    def time_parameterization(self, plan):
        ref_state = self.commander.get_current_state()
        retimed_plan = self.group.retime_trajectory(
            ref_state, plan,
            velocity_scaling_factor=0.1)
        return retimed_plan


    def test_plan_and_time_parameterization(self):
        plan = self.plan()
        retimed_plan = self.time_parameterization(plan)

if __name__ == '__main__':
    PKGNAME = 'moveit_ros_planning_interface'
    NODENAME = 'moveit_test_python_time_parameterization'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonTimeParameterizationTest)

    # suppress cleanup segfault
    os._exit(0)
