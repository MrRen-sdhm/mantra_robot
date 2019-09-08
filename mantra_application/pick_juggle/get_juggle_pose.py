#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from __future__ import print_function
from __future__ import division

import numpy as np
import rospy
import tf
from std_msgs.msg import Float64MultiArray
from transforms3d import quaternions # http://matthew-brett.github.io/transforms3d/


# camera_frame = "kinect2_rgb_optical_frame"
camera_frame = "camera_color_optical_frame"


def callback(msg):
    rects = msg.data
    if len(rects) > 0:
        print "[x y z] [", rects[0], rects[1], rects[2], "]\n"
        br = tf.TransformBroadcaster()
        br.sendTransform((rects[0], rects[1], rects[2]), # (x, y , z)
                    (0, 0, 0, 1), # (x, y, z, w)
                    rospy.Time.now(), "juggle", camera_frame)


# Create a ROS node.
rospy.init_node('get_juggle_pose')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps_yolo/juggle_rects', Float64MultiArray, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(10)
  
while not rospy.is_shutdown():
    rate.sleep()

