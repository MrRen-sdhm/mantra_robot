#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy, sys, tf, tf2_ros

def lookup_trans(from_frame, to_frame):
  print "[INFO] Get transform from %s to %s..." % (from_frame, to_frame)
  while not rospy.is_shutdown():
    try:
      listener = tf.TransformListener()
      listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(1.0))
      (trans, quat) = listener.lookupTransform(from_frame, to_frame, rospy.Time())
      print "[INFO] Transform from %s to %s:" % (from_frame, to_frame)
      print "[%.13f, %.13f, %.13f, %.13f, %.13f, %.13f, %.13f]" % (trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3])
      print "\n\n"
      return (trans, quat)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      pass
    except tf2_ros.TransformException, e:
      print e


if __name__ == "__main__":
  rospy.init_node('get_camera_trans', anonymous=False)
  lookup_trans('camera_color_optical_frame', 'camera_link')
  lookup_trans('ee_link', 'camera_link')



