#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
验证标定结果的精度
真实值获取：手动调节标定钉对准二维码中心，获取机械臂末端位置并转换得到二维码中心点坐标
计算值获取：利用标定所得手眼转换矩阵计算二维码中心点坐标
'''

import time
import glob
import rospy, sys, tf
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from copy import deepcopy
from math import pi
from tf.transformations import *
from transforms3d import quaternions
from geometry_msgs.msg import Pose, PointStamped

import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录

def trans_quat_to_mat44(trans, quat):
  # 平移及四元数数组转齐次变换矩阵
  trans = translation_matrix(trans)
  quat = quaternion_matrix(quat)
  mat44 = np.dot(trans, quat)
  return mat44

def mat44_to_trans_quat(mat44):
  # 齐次变换矩阵转平移及四元数数组
  trans = translation_from_matrix(mat44)
  quat = quaternion_from_matrix(mat44)
  return trans, quat


class image_converter:
  def __init__(self):
    self.cv_image_color = None
    self.cv_image_aruco = None
    self.camera_info = None
    self.bridge = CvBridge()
    self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
    self.aruco_image_sub = rospy.Subscriber("/aruco_tracker/result", Image, self.aruco_callback)
    self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

  def color_callback(self,data):
    try:
        self.cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e

  def aruco_callback(self,data):
    try:
        self.cv_image_aruco = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e

  def camera_info_callback(self,data):
    self.camera_info = data

  def save_camera_info(self, save_path):
    def cal_camera_matrix(k_matrix):
        matrix = np.zeros((3, 3))
        matrix[0, :3] = k_matrix[0:3]
        matrix[1, :3] = k_matrix[3:6]
        matrix[2, :3] = K_matrix[6:9]
        # print "[INFO] Camera intrinsis matrix:\n", matrix
        return matrix

    K_matrix = list(self.camera_info.K)
    matrix = cal_camera_matrix(K_matrix)
    np.savetxt(save_path, matrix)


class aruco_pix_listener:
  def __init__(self):
    self.aruco_pix = None
    self.aruco_pix_sub = rospy.Subscriber("/aruco_tracker/pixel", PointStamped, self.aruco_pix_callback)

  def aruco_pix_callback(self,data):
    self.aruco_pix = [data.point.x, data.point.y]
    # print "[INFO] data", self.aruco_pix


class MantraPickup:
  def __init__(self, mode, t_b_m_trans_real):
    # # 初始化ROS节点
    rospy.init_node('demo', anonymous=True)

    self.mode = mode
    self.t_b_m_trans_real = t_b_m_trans_real
    self.listener = tf.TransformListener()
    self.im_converter = image_converter()
    self.aruco_pix_listener = aruco_pix_listener()
    self.camera_link = "camera_color_optical_frame"
    self.data_num = 0

    while os.path.exists(path + "/data/%s/verify/%06d.color.jpg" % (self.mode, self.data_num)):
      self.data_num += 1

    np.savetxt(path + "/data/%s/verify/%06d.t_b_m_trans_real.txt" % (mode, self.data_num), t_b_m_trans_real)


  def lookup_transform(self, start, end):
    # try:
    #   self.listener.waitForTransform(start, end, rospy.Time.now(), rospy.Duration(5.5)) # 从现在开始0.5s内找不到转换关系，说明此刻转换未发布
    #   (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time(0))
    #   return (cam_trans, cam_quat)
    # except Exception as e:
    #   pass

    while not rospy.is_shutdown():
    # for i in range(10000):  # 若转换关系确实存在，10000次尝试以内应能查询到结果
      try:
        (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time(0))
        # (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time.now())
        return (cam_trans, cam_quat)
      # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      except Exception as e:
        # print e
        # print str(Exception)
        pass
    
    return (None, None)

  def go_to_joint_state(self, joint_goal):
    group = self.group
    group.go(joint_goal, wait=True)


  def cal_marker_pose(self):
    ret = self.lookup_transform(self.camera_link, "aruco_marker")
    print "ret", ret
    if ret != (None, None): # 转换关系存在，即识别到了aruco_marker
      # 保存原始图像
      color_path = path + "/data/%s/verify/%06d.color.jpg" % (self.mode, self.data_num)
      cv2.imwrite(color_path, self.im_converter.cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # 保存坐标系可视化的图像
      aruco_path = path + "/data/%s/verify/%06d.aruco.jpg" % (self.mode, self.data_num)
      cv2.imwrite(aruco_path, self.im_converter.cv_image_aruco, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # 保存marker中心点像素坐标
      pix_path = path + "/data/%s/verify/%06d.pixel.txt" % (self.mode, self.data_num)
      np.savetxt(pix_path, self.aruco_pix_listener.aruco_pix)

      # 保存转换关系，保存为齐次变换矩阵， t_c_m表示从相机到标记的转换矩阵，即标记坐标系相对于相机坐标系的转换矩阵
      trans, quat = self.lookup_transform("base_link", "ee_link") # t_b_e
      t_b_e = trans_quat_to_mat44(trans, quat)
      trans, quat = self.lookup_transform(self.camera_link, "aruco_marker") # t_c_m
      t_c_m = trans_quat_to_mat44(trans, quat)

      t_b_e_path = path + "/data/%s/verify/%06d.t_b_e.txt" % (self.mode, self.data_num)
      np.savetxt(t_b_e_path, t_b_e)

      t_c_m_path = path + "/data/%s/verify/%06d.t_c_m.txt" % (self.mode, self.data_num)
      np.savetxt(t_c_m_path, t_c_m)

      # 载入
      algorithm_ls = ['Tsai-Lenz','Park','Horaud','Andreff','Daniilidis','optimized']
      for algo in algorithm_ls[:]:
        t_e_c = np.loadtxt(path + "/data/%s/t_e_c_%s.txt" % (self.mode, algo))

        t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m)
        error = np.linalg.norm(self.t_b_m_trans_real - translation_from_matrix(t_b_m))

        np.savetxt(path + "/data/%s/verify/%06d.t_b_m_%s.txt" % (mode, self.data_num, algo), t_b_m)
        np.savetxt(path + "/data/%s/verify/%06d.error_%s.txt" % (mode, self.data_num, algo), [error])

        print "\n=================", algo,  "================="
        print "t_b_m:\n", t_b_m, "\n\n", translation_from_matrix(t_b_m)
        print "error:", error
        


if __name__ == "__main__":
  mode = "real"

  '''  ==========================  数据采集  ========================  '''
  # calib_pin_length = 0.07
  # # 填写标定钉顶点指向标定板中心点时，末端的位置即可（末端垂直于桌面向下）
  # # t_b_m_trans_real = [0.53514, 0.00406, 0.32409]
  # # t_b_m_trans_real = [0.50452, 0.0834, 0.32409]
  # t_b_m_trans_real = [0.53491, 0.10376, 0.32409]

  # t_b_m_trans_real[2] -= calib_pin_length

  # mantra_pickup = MantraPickup(mode, t_b_m_trans_real)
  # mantra_pickup.cal_marker_pose()

  # # t_e_c参考值：
  # # Translation: [-0.030, -0.088, 0.053]
  # # Rotation: in Quaternion [-0.006, -0.006, -0.002, 1.000]
  # #           in RPY (radian) [-0.013, -0.013, -0.004]
  # #           in RPY (degree) [-0.719, -0.746, -0.207]

  '''  ==========================  数据分析  ========================  '''

  data_path = path + "/data/%s/verify" % mode

  fls = glob.glob(data_path + "/*.color.jpg")
  # print fls

  algorithm_ls = ['Tsai-Lenz','Andreff','Daniilidis','optimized']

  error_ls_alg = {}
  error_avg_alg = {}
  for alg in algorithm_ls:
    error_ls = []
    for fl in fls[:]:
      prefix = os.path.basename(fl).split(".")[0]
      t_b_m = np.loadtxt(os.path.join(data_path, prefix + ".t_b_m_%s.txt" % alg))
      t_b_m_trans_real = np.loadtxt(os.path.join(data_path, prefix + ".t_b_m_trans_real.txt"))
      error = np.linalg.norm(t_b_m_trans_real - translation_from_matrix(t_b_m))
      error_ls.append(error)
    error_ls_alg[alg] = error_ls
    error_avg_alg[alg] = np.mean(error_ls)

  print error_ls_alg
  # {'optimized': [0.0045119818084148906, 0.00485008689259279, 0.004274458818457337, 0.004416396605187748, 0.005596183946745337, 0.005342350198041011, 0.004628175638331682, 0.0035176374152587277, 0.0046474963618225425, 0.004962882042472168], 'Daniilidis': [0.009496964345304249, 0.010382256746242493, 0.0070284386366037555, 0.009195086541826577, 0.009645216776712385, 0.008881369434118226, 0.009586417985630228, 0.009442373215111512, 0.010139207719229958, 0.010079495661879075], 'Tsai-Lenz': [0.009574731445752568, 0.010462362298350202, 0.007050027098618792, 0.009272774516587165, 0.009749287881677443, 0.008991571659986094, 0.009666498490675999, 0.00949298360530149, 0.010213696647910863, 0.010138428655964541], 'Andreff': [0.009890600501499862, 0.010296423412000636, 0.009318909205008287, 0.009996790436770962, 0.01129483595895493, 0.010664513030008343, 0.009962223680965204, 0.008596999837525859, 0.009686107019729112, 0.009359157692426002]}

  print error_avg_alg
  # {'optimized': 0.004674764972732423, 'Daniilidis': 0.009387682706265846, 'Tsai-Lenz': 0.009461236230082515, 'Andreff': 0.00990665607748892}

  # 绘制误差曲线
  fig = plt.figure(figsize=(9,4.5))

  colors = ['r', 'limegreen', 'deepskyblue', 'orange', 'c']
  marker = "o"
  for idx, alg in enumerate(algorithm_ls):
    error_ls = error_ls_alg[alg]
    if alg == "optimized": marker = "s"
    plt.plot(range(1, len(error_ls)+1), error_ls, "-.", marker=marker, label=alg[0].upper()+alg[1:], color=colors[idx])
    plt.axhline(y=np.mean(error_ls), color=colors[idx], label="%.5f" % np.mean(error_ls))

  # plt.gca().axes.yaxis.set_major_locator(ticker.MultipleLocator(0.001)) # 设置刻度密度
  plt.xticks(range(1, len(error_ls)+1))
  plt.ylim(0.003, 0.013)

  plt.ylabel("error (m)", size=12)
  plt.xlabel("sample", size=12)
  plt.legend(loc="upper left", ncol=len(algorithm_ls), prop={'size': 10})

  plt.savefig(os.path.join(data_path, "error.svg"), dpi=600, bbox_inches='tight')

  plt.show()