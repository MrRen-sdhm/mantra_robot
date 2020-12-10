#!/usr/bin/env python
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np


rospy.init_node("create_cloud_xyzrgb")
pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

reachable_points = np.loadtxt("reachable_points_mode1.csv", delimiter=",")

points = []
for point in reachable_points:
    x = point[0]
    y = point[1]
    z = point[2]
    r = 255
    g = 255
    b = int(z * 255)
    a = 255
    print r, g, b, a
    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
    print hex(rgb)
    pt = [x, y, z, rgb]
    points.append(pt)

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]

print points

header = Header()
header.frame_id = "base_link"
pc2 = point_cloud2.create_cloud(header, fields, points)

while not rospy.is_shutdown():
    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)
    rospy.sleep(1.0)
