#!/usr/bin/env python
#coding=utf-8

import os
import sys
import copy
import math

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
import tf.transformations as tf_trans

import tf2_ros as tf2
import tf2_geometry_msgs as tf2_gm


class ArUcoSub(object):
    def __init__(self, topic_name):
        super(ArUcoSub, self).__init__()

        self.sub_ = rospy.Subscriber(topic_name,
                                     geometry_msgs.msg.PoseStamped,
                                     callback=self.subCallBack,
                                     queue_size=10)

    def subCallBack(self, msg):
        rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("subscrib_demo", anonymous=True)

    demo = ArUcoSub("/aruco_single/pose")

    rospy.loginfo("subscrib /aruco_single/pose")

    rospy.spin()