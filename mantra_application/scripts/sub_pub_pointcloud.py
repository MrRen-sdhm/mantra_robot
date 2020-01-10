#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import thread
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from std_msgs.msg import String

# pub = rospy.Publisher('points', PointCloud2, queue_size=10)

point_cloud = None
update_cloud = True


def callback_pointcloud(data):
    global point_cloud
    assert isinstance(data, PointCloud2)
    # pub.publish(data)

    if update_cloud:
        point_cloud = data

    # gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    # time.sleep(1)
    # print type(gen)
    # for p in gen:
    #   print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])


def talker():
    global point_cloud, update_cloud
    pc = PointCloud2()
    pub = rospy.Publisher('points', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        if isinstance(point_cloud, PointCloud2):
            update_cloud = False
            pub.publish(point_cloud)

            gen = point_cloud2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)
            time.sleep(1)
            print type(gen)
            for p in gen:
              print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])

            # break
        rate.sleep()


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_pointcloud)
    thread.start_new_thread(talker, ())

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        print "mark"
        rate.sleep()
    # talker()
    # rospy.spin()


if __name__ == "__main__":
    main()



