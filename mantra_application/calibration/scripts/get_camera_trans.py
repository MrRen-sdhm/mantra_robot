#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy, sys, tf

def lookup_trans():
  print "[INFO] Get transform..."
  while not rospy.is_shutdown():
    try:
      listener = tf.TransformListener()
      listener.waitForTransform('ee_link', 'camera_link', rospy.Time(), rospy.Duration(1.0))
      (trans, quat) = listener.lookupTransform('/ee_link', '/camera_link', rospy.Time())
      print "[INFO] Transform from ee_link to camera_link:"
      print "[%.13f, %.13f, %.13f, %.13f, %.13f, %.13f, %.13f]" % (trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3])
      return (trans, quat)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      pass


if __name__ == "__main__":
  rospy.init_node('get_camera_trans', anonymous=False)
  lookup_trans()


