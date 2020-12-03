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


class ArUco(object):
    def __init__(self, target_frame, source_frame):
        super(ArUco, self).__init__()
        self.listener_ = tf.TransformListener()
        self.target_frame_ = target_frame
        self.source_frame_ = source_frame
        self.max_time_ = 1

    def find(self):
        is_find = False
        position = [0, 0, 0]
        orientation = [0, 0, 0, 1]

        try:
            self.listener_.waitForTransform(self.target_frame_,
                                            self.source_frame_, rospy.Time(0),
                                            rospy.Duration(self.max_time_))

            (position, orientation) = self.listener_.lookupTransform(
                self.target_frame_, self.source_frame_, rospy.Time(0))
            is_find = True
        except:
            is_find = False

        print((position, orientation))
        return (is_find, position, orientation)

    def setMaxTime(self, max_time):
        self.max_time_ = max_time
