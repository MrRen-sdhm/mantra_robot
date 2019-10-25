#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 获取当前相机位姿及拍摄的RGB和对齐的深度图，并保存到data文件夹

import rospy, sys, tf
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from tf.transformations import quaternion_from_euler
from transforms3d import quaternions
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import os
path = os.path.dirname(os.path.dirname(__file__)) # 获取文件所在目录的上级目录
print "path:", path

GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'base_link'

cv_image_color = None
cv_image_depth = None


class ImageReceiver:
    def __init__(self):
        self.info_save_cnt = 0
        # 创建cv_bridge，声明图像的订阅者
        self.bridge = CvBridge()
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)
        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)

    def info_callback(self,data):
        if self.info_save_cnt == 0:
            matrix = np.zeros((3, 3))
            matrix[0,:3] = data.K[0:3]
            matrix[1,:3] = data.K[3:6]
            matrix[2,:3] = data.K[6:9]
            print "[INFO] Camera intrinsis matrix:\n", matrix
            np.savetxt(path + "/data/camera-intrinsics.txt", matrix)
            self.info_save_cnt = 1

    def color_callback(self,data):
        global cv_image_color
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # RGB8
            cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        # cv2.imshow("Image Color", cv_image_color)
        # cv2.waitKey(3)

    def depth_callback(self,data):
        global cv_image_depth
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            # print data.encoding # 16UC1
            cv_image_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print e

        # 显示Opencv格式的图像
        # cv2.imshow("Image Depth", cv_image_depth)
        # cv2.waitKey(3)


class PoseImageSaver:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('get_camera_pose_image_auto')

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        arm.set_max_velocity_scaling_factor(0.8)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_planning_time(0.5) # 规划时间限制为2秒
        arm.allow_replanning(False) # 当运动规划失败后，是否允许重新规划

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # 获取终端link的名称
        eef_link = arm.get_end_effector_link()

        # 开始图像接收
        image_receiver = ImageReceiver()

        self.group = arm
        self.eef_link = eef_link
        self.reference_frame = reference_frame
        self.moveit_commander = moveit_commander
        self.listener = tf.TransformListener()
        self.save_cnt = 0
        print "\n\n[INFO] Pose image saver started."

    def goto_pose_named(self, pose_name):
        group = self.group
        group.set_named_target(pose_name)
        group.go()

    def camera_pose_get(self):
        while not rospy.is_shutdown():
            try:
                (position, orientation) = self.listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))
                rospy.loginfo("Camera pose reference to base_link:\nposition:\n %s\norientation:\n %s\n",
                    str(position), str(orientation))
                return position, orientation
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def cal_camera_pose(self):
        # 获取相机位姿
        print "[INFO] Get camera pose onece."
        camera_position, camera_orientation = self.camera_pose_get()
        orix, oriy, oriz, oriw = camera_orientation
        # print orix, oriy, oriz, oriw

        # 四元数转旋转矩阵
        quat_wxyz = (oriw, orix, oriy, oriz)
        rotation_matrix = quaternions.quat2mat(quat_wxyz)
        # print rotation_matrix

        # 创建齐次变换矩阵
        matrix = np.zeros((4, 4))
        matrix[:3,:3] = rotation_matrix
        matrix[:3,3] = np.array(camera_position).T
        matrix[3][3] = 1.0
        print "[INFO] Camera pose matrix:\n", matrix
        return matrix

    def save_pose_image(self):
        global cv_image_color, cv_image_depth
        cnt = self.save_cnt
        print "[INFO] Saved data Cnt:", cnt, "\n\n"
        # 保存相机位姿
        matrix = self.cal_camera_pose()
        np.savetxt(path + "/data/frame-%06d.pose.txt" % cnt, matrix)

        # 保存RGB及深度图
        cv2.imwrite(path + "/data/frame-%06d.color.jpg" % cnt, cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
        cv2.imwrite(path + "/data/frame-%06d.depth.png" % cnt, cv_image_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        self.save_cnt += 1


if __name__ == "__main__":
    pose_image_saver = PoseImageSaver()

    # # 机械臂移动并采集相机位姿及图像数据
    # pose_image_saver.goto_pose_named("test_1")
    # pose_image_saver.save_pose_image()

    # pose_image_saver.goto_pose_named("test_2")
    # pose_image_saver.save_pose_image()

    for i in range(50):
        pose_image_saver.save_pose_image()

    # 调用Python脚本进行三维重建
    print "\n\n[INFO] Run python script to reconstruct the mesh."
    data_cnt = pose_image_saver.save_cnt
    fusion_exe_path = os.path.join(os.path.dirname(__file__), 'fusion.py')
    command = "python " + fusion_exe_path
    images_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
    os.system("%s %s %s" % (command, images_dir, str(data_cnt)))
