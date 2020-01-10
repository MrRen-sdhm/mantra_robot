#!/usr/bin/env python
# -*- coding: utf-8 -*-

# https://www.programcreek.com/python/example/103984/tf.transformations.quaternion_matrix

import tf
import rospy
from tf.transformations import *
from transforms3d import quaternions

import numpy as np

rot = np.array([[-0.13353289, -0.62635965, 0.76801208],
                [-0.98501484, -0.00147204, -0.1724633],
                [0.1091546, -0.77953282, -0.61677699]])

quat = np.array([-0.60924464,  0.66121816, -0.35994025,  0.24910745])

trans = np.array([0.07758124, 0.13366812, 0.40189549])

""" #####################  使用 ros transformations  ##################### """

# 转换为齐次变换矩阵
trans = translation_matrix(trans)
quat = quaternion_matrix(quat)

print "转换为齐次变换矩阵"
print trans
print quat

# 将旋转平移表示为一个齐次变换矩阵
mat44 = np.dot(trans, quat)

print "将旋转平移表示为一个齐次变换矩阵"
print mat44

# 转换回普通矩阵
trans = translation_from_matrix(trans)
quat = quaternion_from_matrix(quat)

print "转换为齐次变换矩阵"
print trans
print quat

""" #####################  使用transforms3d  ##################### """
# 将旋转矩阵转换为四元数
quat = quaternions.mat2quat(rot)  # [w, x, y, z]
quat[[3, 0]] = quat[[0, 3]]  # [x, y, z, w]

print "将旋转矩阵转换为四元数"
print quat

# 将将四元数转换为旋转矩阵
quat[[3, 0]] = quat[[0, 3]]  # [w, x, y, z]
rot = quaternions.quat2mat(quat)
print "将四元数转换为旋转矩阵"
print rot

""" #####################  测试世界坐标系->相机坐标系->物体坐标系  ##################### """

'''世界->相机'''
base2cam_trans = [0.2525506078699997, 0.02896931482271796, 0.732187019393393]
base2cam_quat = [-0.6794260474751038, 0.6746756895702364, -0.19979275869284494, 0.20802799357178542]

'''相机->物体'''
cam2obj_rot = np.array([[-0.00765193, 0.31202572, - 0.95004284],
                        [0.40310443, 0.87041852, 0.28262771],
                        [0.91512199, - 0.38080383, - 0.13243933]])

cam2obj_trans = [0.05185268692428623, 0.03499475249100164, 0.34325048453803747]


def cal_obj_pose_in_world(base2cam_trans, base2cam_quat, cam2obj_trans, cam2obj_rot):
    """
    1、ROS transforms 没有提供旋转矩阵与四元数之间转换的函数，但是提供了齐次变换矩阵(homogeneous matrix)转换为四元数的函数，
    可以先创建齐次变换矩阵表示的旋转矩阵，再通过函数将旋转矩阵转换为四元数。
    2、通过齐次变换矩阵(homogeneous matrix)可实现坐标系的变换。
    """
    # 将旋转矩阵转换为四元数
    cam2obj_rot44 = numpy.identity(4)  # 创建齐次变换矩阵
    cam2obj_rot44[:3, :3] = cam2obj_rot  # 写入旋转矩阵
    cam2obj_quat = quaternion_from_matrix(cam2obj_rot44)

    # cam2obj转换为齐次变换矩阵
    cam2obj_trans = translation_matrix(cam2obj_trans)
    cam2obj_quat = quaternion_matrix(cam2obj_quat)
    cam2obj_mat44 = np.dot(cam2obj_trans, cam2obj_quat)

    '''世界->物体'''
    # base2cam转换为齐次变换矩阵
    base2cam_trans44 = translation_matrix(base2cam_trans)
    base2cam_quat44 = quaternion_matrix(base2cam_quat)
    base2cam_mat44 = np.dot(base2cam_trans44, base2cam_quat44)

    # base坐标系下物体位姿
    base2obj_mat44 = np.dot(base2cam_mat44, cam2obj_mat44)  # base->cam->obj
    base2obj_trans = translation_from_matrix(base2obj_mat44)
    base2obj_quat = quaternion_from_matrix(base2obj_mat44)

    return base2obj_trans, base2obj_quat


rospy.init_node('transform test', anonymous=True)
broadcaster = tf.TransformBroadcaster()

base2obj_trans, base2obj_quat = cal_obj_pose_in_world(base2cam_trans, base2cam_quat, cam2obj_trans, cam2obj_rot)

while not rospy.is_shutdown():
    rospy.sleep(1)
    broadcaster.sendTransform(base2cam_trans, base2cam_quat, rospy.Time.now(), "camera", "world")
    broadcaster.sendTransform(base2obj_trans, base2obj_quat, rospy.Time.now(), "obj", "world")

