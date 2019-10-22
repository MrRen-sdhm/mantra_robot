#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 获取当前相机位姿及拍摄的RGB和对齐的深度图，并保存到data文件夹

import rospy, sys, tf
import moveit_commander
import geometry_msgs.msg
from transforms3d import quaternions
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import os
path = os.path.abspath(os.path.dirname(os.getcwd())) # 上级目录

GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'base_link'

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


class MoveItDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_pick_and_place_demo')
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.arm = arm
 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame(REFERENCE_FRAME)

    
    def camera_pose_listener(self):
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


if __name__ == "__main__":
    demo = MoveItDemo()
    rospy.sleep(5)
    num = 0

    while True:
        s = raw_input("type in 's' to get camera pose:")
        if s is 's':
            # 获取相机位姿
            print "get camera pose..."
            camera_position, camera_orientation = demo.camera_pose_listener()
            orix, oriy, oriz, oriw = camera_orientation
            print num, orix, oriy, oriz, oriw

            # 四元数转旋转矩阵
            quat_wxyz = (oriw, orix, oriy, oriz)
            rotation_matrix = quaternions.quat2mat(quat_wxyz)
            print rotation_matrix

            # 创建齐次变换矩阵
            matrix = np.zeros((4, 4))
            matrix[:3,:3] = rotation_matrix
            matrix[:3,3] = np.array(camera_position).T
            matrix[3][3] = 1.0
            print matrix

            # 保存相机位姿
            np.savetxt(path + "/data/frame-%06d.pose.txt" % num, matrix)

            # 保存RGB及深度图
            cv2.imwrite(path + "/data/frame-%06d.color.jpg", cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
            cv2.imwrite(path + "/data/frame-%06d.depth.png", cv_image_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

            num += 1
