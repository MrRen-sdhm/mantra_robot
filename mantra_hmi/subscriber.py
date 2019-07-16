#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  :

import rospy
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray


def callback(data):
    rospy.loginfo(data.data)


def listener():
    rospy.init_node('mantra_control_sub')
    rospy.Subscriber("mantra_hmi", Int32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
