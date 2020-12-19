#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import glob
import rospy, sys, tf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

from copy import deepcopy
from math import pi
from tf.transformations import *
from transforms3d import quaternions


path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录
data_path = path + "/data"

fls = glob.glob(data_path + "/*.color.jpg")

camera_info = np.loadtxt(os.path.join(data_path, "camera_info.txt"))
print "K", camera_info

M = np.zeros((3,4))
M[:3,:3] = camera_info

print "M", M

t_e_c = np.loadtxt(os.path.join(data_path, "t_e_c_Tsai-Lenz.txt"))

# ===================================  测试像素坐标+深度->世界坐标  =============================
# fx = camera_info[0][0]
# fy = camera_info[1][1]
# u0 = camera_info[0][2]
# v0 = camera_info[1][2]

# z = 0.226152792573
# u = 317 # pix_x
# v = 293 # pix_y
# x = z*(u-u0)/fx
# y = z*(v-v0)/fy

# print x, y

# =====================================  测试世界坐标->像素坐标  ==============================
def world2pixel(M, x, y, z):
  x = np.dot(M, np.array([[x], [y], [z], [1]]))

  zc = x[2]

  u = x[0] / zc
  v = x[1] / zc
  return [u, v]

# [u, v] = world2pixel(M, -0.0428294315934,-0.0150737017393,0.353678137064)
# print u
# print v
# exit()


# ==========================================================================================



t_b_m_avg = np.zeros((4,4))

# 第一次循环，计算t_b_m_avg
for fl in fls:
  prefix = os.path.basename(fl).split(".")[0]

  t_b_e = np.loadtxt(os.path.join(data_path, prefix + ".t_b_e.txt"))
  t_c_m = np.loadtxt(os.path.join(data_path, prefix + ".t_c_m.txt"))

  t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m) # 基坐标系->marker（使用当前坐标关系计算得到的值）

  t_b_m_avg += t_b_m

t_b_m_avg = t_b_m_avg / (len(fls))  # 将均值作为理想值
print "t_b_m_avg", t_b_m_avg


# 第二次循环，计算重投影
for fl in fls:
  prefix = os.path.basename(fl).split(".")[0]
  print "\n", prefix

  pixel = np.loadtxt(os.path.join(data_path, prefix + ".pixel.txt"))
  print pixel

  t_b_e = np.loadtxt(os.path.join(data_path, prefix + ".t_b_e.txt"))
  print t_b_e

  t_c_m = np.loadtxt(os.path.join(data_path, prefix + ".t_c_m.txt"))
  print "t_c_m real", t_c_m

  # t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m) # 基坐标系->marker（使用当前坐标关系计算得到的值）
  t_b_m = t_b_m_avg # 基坐标系->marker （使用理论值/均值）

  t_e_b = np.linalg.inv(t_b_e)
  t_c_e = np.linalg.inv(t_e_c)

  # aruco_ros发布的中心点像素坐标不准确，这里利用世界坐标转换得到中心点像素坐标
  pixel = world2pixel(M, t_c_m[0][3], t_c_m[1][3], t_c_m[2][3])
  print "[INFO] Standard:[%.6f, %.6f]" % (pixel[0], pixel[1])

  print "=============== 方式1 根据点云生成公式（将相机坐标系定义为世界坐标系）"
  # 利用坐标关系计算t_c_m
  # t_c_m = t_c_e*t_e_b*t_b_m
  t_c_m = np.dot(np.dot(t_c_e, t_e_b), t_b_m)

  print "t_c_m cal", t_c_m
  # M * ([xc, yc, zc, 1].T)
  x = np.dot(M, t_c_m[:, 3])

  zc = x[2]

  # 重投影的marker中心点坐标
  u = x[0] / zc
  v = x[1] / zc

  # print zc
  # print u
  # print v


  print "=============== 方式2 将marker坐标系定义为世界坐标系"
  # M*t_c_e*t_e_b*t_b_m*([0,0,0,1].T)
  x = np.dot(M, t_c_e)
  x = np.dot(x, t_e_b)
  x = np.dot(x, t_b_m)
  x = np.dot(x, np.array([[0], [0], [0], [1]]))

  zc = x[2][0]

  u = x[0][0] / zc
  v = x[1][0] / zc

  print "[INFO] Reprojection:[%.6f, %.6f]" % (u, v)
  # print zc
  # print u
  # print v

  # 可视化
  image_color = mpimg.imread(os.path.join(data_path, prefix + ".color.jpg"))
  image_aruco = mpimg.imread(os.path.join(data_path, prefix + ".aruco.jpg"))

  fig1 = plt.figure(figsize=(12,6))
  plt.subplots_adjust(top=1,bottom=0,left=0,right=1,hspace=0,wspace=0.01)

  ax1 = plt.subplot(1, 2, 1)
  ax1.plot(u,v,'go', markersize=1) # 重投影点
  ax1.plot(pixel[0],pixel[1],'ro', markersize=1)  # 真实点
  ax1.imshow(image_color)
  ax1.axis('off')

  ax1 = plt.subplot(1, 2, 2)
  # fig2 = plt.figure()
  ax1.imshow(image_aruco)
  ax1.axis('off')

  plt.show()

