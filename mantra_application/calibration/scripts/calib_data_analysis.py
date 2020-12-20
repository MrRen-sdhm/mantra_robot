#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import glob
import math
import rospy, sys, tf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D

from copy import deepcopy
from math import pi
from tf.transformations import *
from transforms3d import quaternions

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


path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录
data_path = path + "/data"

fls = glob.glob(data_path + "/*.color.jpg")

camera_info = np.loadtxt(os.path.join(data_path, "camera_info.txt"))
print "K", camera_info

M = np.zeros((3,4))
M[:3,:3] = camera_info

print "M", M

# 末端->相机的真实转换关系
t_e_c_trans_real = [-0.07, 0, 0.0345]
t_e_c_euler_real = [0, 0, -math.pi/2]

# 基坐标系->marker的真实转换关系
t_b_m_trans_real = [0.45, 0, 0]
t_b_m_euler_real = [pi/2, 0, pi/2]

algorithm_ls = ['Tsai-Lenz','Park','Horaud','Andreff','Daniilidis']
for alg in algorithm_ls[:]:
  t_e_c = np.loadtxt(os.path.join(data_path, "t_e_c_%s.txt" % alg))

  # 计算与真实值的差别
  print "================= %s" % alg
  print t_e_c_trans_real
  print translation_from_matrix(t_e_c)
  print numpy.linalg.norm(t_e_c_trans_real - translation_from_matrix(t_e_c))

  print t_e_c_euler_real
  print euler_from_matrix(t_e_c)
  print numpy.linalg.norm(t_e_c_euler_real - np.array(euler_from_matrix(t_e_c)))
  # exit()
  continue

  # =====================================  第一次循环，计算t_b_m_avg  ==============================
  t_b_m_avg = np.zeros((4,4))
  for fl in fls:
    prefix = os.path.basename(fl).split(".")[0]

    t_b_e = np.loadtxt(os.path.join(data_path, prefix + ".t_b_e.txt"))
    t_c_m = np.loadtxt(os.path.join(data_path, prefix + ".t_c_m.txt"))

    t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m) # 基坐标系->marker（使用当前坐标关系计算得到的值）

    t_b_m_avg += t_b_m

  t_b_m_avg = t_b_m_avg / (len(fls))  # 将均值作为理想值
  print "t_b_m_avg", t_b_m_avg

  
  # =======================================  第二次循环，计算重投影  ===============================
  reprojection_error_ls = []
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

    reprojection_error = math.sqrt((pixel[0]-u)**2 + (pixel[1]-v)**2)
    print "[INFO] Reprojection error:", reprojection_error

    reprojection_error_ls.append(reprojection_error)

    # 可视化
    # image_color = mpimg.imread(os.path.join(data_path, prefix + ".color.jpg"))
    # image_aruco = mpimg.imread(os.path.join(data_path, prefix + ".aruco.jpg"))

    # fig1 = plt.figure(figsize=(12,6))
    # plt.subplots_adjust(top=1,bottom=0,left=0,right=1,hspace=0,wspace=0.05)

    # ax1 = plt.subplot(1, 2, 1)
    # ax1.plot(u,v,'go', markersize=3) # 重投影点
    # ax1.plot(pixel[0],pixel[1],'ro', markersize=3)  # 真实点
    # ax1.imshow(image_color)
    # ax1.axis('off')

    # ax2 = plt.subplot(1, 2, 2)
    # ax2.imshow(image_aruco)
    # ax2.axis('off')

    # plt.show()


  # 绘制重投影误差直方图
  error_avg = np.array(reprojection_error_ls).mean()
  print "error_avg", error_avg

  fig = plt.figure(figsize=(10,6))
  # plt.plot(range(len(reprojection_error_ls)), reprojection_error_ls, "r.")
  plt.barh(range(len(reprojection_error_ls)), reprojection_error_ls, height=0.7, color='steelblue', alpha=0.8)

  plt.axvline(x=error_avg, color="r")
  plt.text(error_avg + 0.02, 4, '<-- Mean (%.3f)' % error_avg, size=11, color="r")

  for x, y in enumerate(reprojection_error_ls):
      plt.text(y + 0.01, x - 0.3, '%.3f' % y)

  plt.gca().axes.yaxis.set_major_locator(ticker.MultipleLocator(2)) # 设置刻度密度
  # plt.ylim(0, len(reprojection_error_ls))
  plt.xlim(0, max(reprojection_error_ls) + 0.2)

  plt.xlabel("Reprojection error (pix)", size=12)
  plt.ylabel("Sample", size=12)

  plt.savefig(os.path.join(data_path, "Reprojection error - %s.svg" % alg), dpi=600, bbox_inches='tight')
  plt.show()

