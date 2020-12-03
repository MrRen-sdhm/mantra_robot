#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 获取当前相机位姿及拍摄的RGB和对齐的深度图，并保存到data文件夹

import rospy, sys, tf
import yaml
import numpy as np
import geometry_msgs.msg
from transforms3d import quaternions
from geometry_msgs.msg import PoseStamped, Pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import os
path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录
# save_path = path + "/data/multi-view/"
save_path = "./"

GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'base_link'


def save_camera_info(msg, save_path):
    def cal_camera_matrix(k_matrix):
        matrix = np.zeros((3, 3))
        matrix[0, :3] = k_matrix[0:3]
        matrix[1, :3] = k_matrix[3:6]
        matrix[2, :3] = K_matrix[6:9]
        # print "[INFO] Camera intrinsis matrix:\n", matrix
        return matrix

    K_matrix = list(msg.K)
    matrix = cal_camera_matrix(K_matrix)
    np.savetxt(save_path, matrix)


class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的订阅者
        self.cv_image_color = None
        self.cv_image_depth = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.camera_info_sub = rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_callback)

    def color_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # RGB8
            self.cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        # cv2.imshow("Image Color", self.cv_image_color)
        # cv2.waitKey(3)

    def depth_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # 16UC1
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        # cv2.imshow("Image Depth", self.cv_image_depth)
        # cv2.waitKey(3)

    def camera_info_callback(self,data):
        self.camera_info = data

    
def camera_pose_listener():
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (position, orientation) = listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))
            rospy.loginfo("Camera pose reference to base_link:\nposition:\n %s\norientation:\n %s\n",
                str(position), str(orientation))
            return position, orientation
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def get_camera_pose_image(im_converter, num, save_path, min_z=-1):
    print "get camera pose..."
    camera_position, camera_orientation = camera_pose_listener()
    orix, oriy, oriz, oriw = camera_orientation
    # print num, orix, oriy, oriz, oriw

    # 四元数转旋转矩阵
    quat_wxyz = (oriw, orix, oriy, oriz)
    rotation_matrix = quaternions.quat2mat(quat_wxyz)
    # print rotation_matrix

    # 创建齐次变换矩阵
    matrix = np.zeros((4, 4))
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = np.array(camera_position).T
    matrix[3][3] = 1.0
    # print matrix

    # 保存相机位姿
    pose_path = save_path + "/frame-%06d.pose.txt" % num
    np.savetxt(pose_path, matrix)
    # 保存相机内参
    camera_info_path = save_path
    save_camera_info(im_converter.camera_info, camera_info_path + "/camera-intrinsics.txt")

    # 保存RGB及深度图
    color_path = save_path + "/frame-%06d.color.jpg" % num
    depth_path = save_path + "/frame-%06d.depth.png" % num
    cv2.imwrite(color_path, im_converter.cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    cv2.imwrite(depth_path, im_converter.cv_image_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

    # 生成点云
    cloud_path = save_path + "/frame-%06d.cloud.pcd" % num
    create_cloud_exe_path = path + "/post_process/build/create_point_cloud"
    command = "%s %s %s %s %s %s -1 1 -1 1 -1 1" % (
    create_cloud_exe_path, color_path, depth_path, camera_info_path, pose_path, cloud_path)
    os.system(command)

    # 生成桌面以上点云
    if min_z > -1:
        cloud_path = save_path + "/frame-%06d.cloud_seg.pcd" % num
        command = "%s %s %s %s %s %s -1 1 -1 1 %f 1" % (
            create_cloud_exe_path, color_path, depth_path, camera_info_path, pose_path, cloud_path, min_z)
        os.system(command)

    # 转换为ply
    ply_path = save_path + "/frame-%06d.cloud.ply" % num
    command = "pcl_pcd2ply %s %s" % (cloud_path, ply_path)
    os.system(command)


if __name__ == "__main__":

    rospy.init_node('get_camera_pose_image_manual')
    im_converter = image_converter()
    rospy.sleep(1)
    num = 0

    while not rospy.is_shutdown():
        s = raw_input("press enter to get camera pose:")
        get_camera_pose_image(im_converter, num, save_path)
        num += 1
