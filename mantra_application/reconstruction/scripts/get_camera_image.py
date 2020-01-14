#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

cv_image_color = None
cv_image_depth = None

class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的订阅者
        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)

    def color_callback(self,data):
        global cv_image_color
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # RGB8
            cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite("cv_image_color.jpg", cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        cv2.imshow("Image Color", cv_image_color)
        cv2.waitKey(3)

    def depth_callback(self,data):
        global cv_image_depth
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # 16UC1
            cv_image_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
            cv2.imwrite("cv_image_depth.png", cv_image_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        # cv2.imshow("Image Depth", cv_image_depth)
        # cv2.waitKey(3)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()