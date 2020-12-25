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
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib as mpl

from copy import deepcopy
from math import pi
from tf.transformations import *
from transforms3d import quaternions

from matplotlib.font_manager import FontProperties
from matplotlib import rcParams

# 全局设置字体及大小，设置公式字体即可，若要修改刻度字体，可在此修改全局字体
config = {
    "mathtext.fontset":'stix',
}
rcParams.update(config)

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

# sim
# result of Tsai-Lenz: [-0.07160164560846347, -0.0011253596536365436, 0.031513237490110285] [-0.0004284022387325089, 0.0022777350290372755, -0.7063960978547398, 0.707812956458312]
# result of Park: [-0.07160145637172599, -0.0011667813296825577, 0.03152371428276636] [-0.000177749527278495, 0.0020999935784514756, -0.7064286616916676, 0.7077811133200683]
# result of Horaud: [-0.07160128768157017, -0.0011616251967257445, 0.031521509277004665] [-0.0001895128150157786, 0.002120060211031727, -0.7064282271194045, 0.7077814841874082]
# result of Andreff: [-0.07205708963826246, -0.00032957470631941876, 0.034356712114669785] [-0.00025540544303507103, 0.0020301570432484928, -0.706446608980315, 0.70776337987399]
# result of Daniilidis: [-0.07198965258711985, -0.0011606763930636893, 0.03156098228350543] [-0.0005500575954816343, 0.002079122054706506, -0.7065332113392672, 0.7076766182111326]

# real
# result of Tsai-Lenz: [-0.03748010130548035, -0.09629382866311062, 0.04867988527310136] [-0.011702659800261781, -0.0071545773494036125, 0.00602671853845912, 0.9998877629215235]
# result of Park: [-0.03748018957186601, -0.09629620643608905, 0.048679186009584124] [-0.011707650774110142, -0.007171784589512509, 0.006035788327611953, 0.9998875265140661]
# result of Horaud: [-0.03748156561747055, -0.09629465983236554, 0.04868020922790178] [-0.011705148838259499, -0.007153327365567064, 0.006033499169702405, 0.9998877018375837]
# result of Andreff: [-0.036684769921625476, -0.09826050927410673, 0.06144543683136272] [-0.011585944497666967, -0.007361641666711494, 0.005729814681024802, 0.9998893645527933]
# result of Daniilidis: [-0.037172288585535916, -0.09611992626082094, 0.04860023217490725] [-0.011604073612032877, -0.007543602738283028, 0.005313086830818958, 0.999890099281747]


""" ==========================================  准备工作 ======================================= """
mode = "real" # ["sim", "real"]

path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录
data_path = path + "/data/%s" % mode

fls = glob.glob(data_path + "/*.color.jpg")

camera_info = np.loadtxt(os.path.join(data_path, "camera_info.txt"))
print "K", camera_info

M = np.zeros((3,4))
M[:3,:3] = camera_info

print "M", M

# 载入t_b_e和t_c_m，并保存到字典
t_b_e_dict, t_c_m_dict = {}, {}
for fl in fls:
  prefix = os.path.basename(fl).split(".")[0]

  # pixel = np.loadtxt(os.path.join(data_path, prefix + ".pixel.txt"))
  t_b_e = np.loadtxt(os.path.join(data_path, prefix + ".t_b_e.txt"))
  t_c_m = np.loadtxt(os.path.join(data_path, prefix + ".t_c_m.txt"))

  t_b_e_dict[prefix] = t_b_e
  t_c_m_dict[prefix] = t_c_m

if mode == "sim":
  # 末端->相机的真实转换关系
  t_e_c_trans_real = [-0.07, 0, 0.0345]
  t_e_c_euler_real = [0, 0, -math.pi/2]
  t_e_c_quat_real = [0.000, -0.000, -0.707, 0.707]

  # 计算齐次变换矩阵
  trans = translation_matrix(t_e_c_trans_real)
  quat = quaternion_matrix(quaternion_from_euler(t_e_c_euler_real[0], t_e_c_euler_real[1], t_e_c_euler_real[2]))
  t_e_c_real = np.dot(trans, quat) # [[ 0.      1.      0.     -0.07  ]  [-1.      0.      0.      0.    ] [ 0.      0.      1.      0.0345] [ 0.      0.      0.      1.    ]]

  # 基坐标系->marker的真实转换关系
  t_b_m_trans_real = [0.45, 0, 0.001]
  t_b_m_euler_real = [pi/2, 0, pi/2]

  # 计算齐次变换矩阵
  trans = translation_matrix(t_b_m_trans_real)
  quat = quaternion_matrix(quaternion_from_euler(t_b_m_euler_real[0], t_b_m_euler_real[1], t_b_m_euler_real[2]))
  t_b_m_real = np.dot(trans, quat) # [[ 0.00000000e+00 -1.11022302e-16  1.00000000e+00  4.50000000e-01] [ 1.00000000e+00 -2.22044605e-16 -1.11022302e-16  0.00000000e+00] [ 0.00000000e+00  1.00000000e+00  0.00000000e+00  1.00000000e-03] [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

elif mode == "real":
  t_b_m_trans_real = np.loadtxt(os.path.join(data_path, "t_b_m_trans_real.txt"))


def cal_reprojection_error(t_e_c, t_b_m_avg, log=True):
  reprojection_error_ls = []
  for fl in fls:
    prefix = os.path.basename(fl).split(".")[0]
    if log: print "\n", prefix

    t_b_e = t_b_e_dict[prefix]
    t_c_m = t_c_m_dict[prefix]

    if mode == "sim":
      # 计算or载入t_b_m
      # t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m) # 基坐标系->marker（使用当前坐标关系计算得到的值）
      # t_b_m = t_b_m_avg # 基坐标系->marker （使用理论值/均值）
      t_b_m = t_b_m_real # 载入t_b_m的真实值，包括旋转和平移部分
      t_b_m[:,:3] = t_b_m_avg[:,:3]
    elif mode == "real":
      t_b_m = translation_matrix(t_b_m_trans_real)
      t_b_m[:,:3] = t_b_m_avg[:,:3] # 旋转部分，即前3列使用均值(适合真实环境)

    t_e_b = np.linalg.inv(t_b_e)
    t_c_e = np.linalg.inv(t_e_c)

    # aruco_ros发布的中心点像素坐标不准确，这里利用世界坐标转换得到中心点像素坐标
    pixel = world2pixel(M, t_c_m[0][3], t_c_m[1][3], t_c_m[2][3])
    # print "[INFO] Standard:[%.6f, %.6f]" % (pixel[0], pixel[1])

    # print "=============== 方式1 根据点云生成公式（将相机坐标系定义为世界坐标系）"
    # # 利用坐标关系计算t_c_m
    # # t_c_m = t_c_e*t_e_b*t_b_m
    # t_c_m = np.dot(np.dot(t_c_e, t_e_b), t_b_m)

    # print "t_c_m cal", t_c_m
    # # M * ([xc, yc, zc, 1].T)
    # x = np.dot(M, t_c_m[:, 3])

    # zc = x[2]

    # # 重投影的marker中心点坐标
    # u = x[0] / zc
    # v = x[1] / zc

    # print "=============== 方式2 将marker坐标系定义为世界坐标系"
    # M*t_c_e*t_e_b*t_b_m*([0,0,0,1].T)
    x = np.dot(M, t_c_e)
    x = np.dot(x, t_e_b)
    x = np.dot(x, t_b_m)
    x = np.dot(x, np.array([[0], [0], [0], [1]]))

    zc = x[2][0]

    u = x[0][0] / zc
    v = x[1][0] / zc

    # print "[INFO] Reprojection:[%.6f, %.6f]" % (u, v)

    reprojection_error = np.linalg.norm(np.array(pixel).T - np.array([u,v]).T) # math.sqrt((pixel[0]-u)**2 + (pixel[1]-v)**2)
    if log: print "[INFO] Reprojection error:", reprojection_error
    reprojection_error_ls.append(reprojection_error)

    # 可视化重投影点
    # image_color = mpimg.imread(os.path.join(data_path, prefix + ".color.jpg"))
    # image_aruco = mpimg.imread(os.path.join(data_path, prefix + ".aruco.jpg"))

    # fig1 = plt.figure(figsize=(12,6))
    # plt.subplots_adjust(top=1,bottom=0,left=0,right=1,hspace=0,wspace=0.05)

    # ax1 = plt.subplot(1, 2, 1)
    # ax1.plot(u,v,'co', markersize=3) # 重投影点
    # ax1.plot(pixel[0],pixel[1],'ro', markersize=3)  # 真实点
    # ax1.imshow(image_color)
    # ax1.axis('off')

    # ax2 = plt.subplot(1, 2, 2)
    # ax2.imshow(image_aruco)
    # ax2.axis('off')

    # plt.show()

  return reprojection_error_ls

""" ==========================================  核心代码 ======================================= """
algorithm_ls = ['Tsai-Lenz','Horaud','Andreff','Daniilidis'] # ,'Park'
# algorithm_ls = ['Tsai-Lenz','optimized']
algorithm_ls = ['Tsai-Lenz','Andreff','Daniilidis','optimized'] # ,'Park'
reprojection_error_alg = {}
reprojection_error_ls_alg = []
t_b_m_trans_ls_alg = {}
t_b_m_euler_ls_alg = {}
for alg in algorithm_ls[:]:
  t_e_c = np.loadtxt(os.path.join(data_path, "t_e_c_%s.txt" % alg))
  x, y, z = translation_from_matrix(t_e_c)
  roll, pitch, yall = euler_from_matrix(t_e_c)

  # 修改标定值
  # t_e_c[0][3] = -0.07048943296140564
  # t_e_c[1][3] = 0.0005451858029217682
  # t_e_c[2][3] = 0.034282388258738056

  # t_e_c[0][3] = t_e_c_trans_real[0]
  # t_e_c[1][3] = t_e_c_trans_real[1]
  # t_e_c[2][3] = t_e_c_trans_real[2]

  # 载入t_e_c的真实值，理应使得重投影误差最小！！！
  # 2.9347640693972408
  # t_e_c = t_e_c_real # 1.4045797576453039
  # t_e_c[:,3] = t_e_c_real[:,3] # 仅载入平移矩阵 1.4349794388356325

  # print "t_e_c", t_e_c
  # print t_e_c[:3,3] # [-0.07058943 -0.00085481  0.03258239]  [-0.07, 0, 0.0345]

  # [-0.07048943296140564, 0.0005451858029217682, 0.034282388258738056] best
  # error_min: 0.2656618045945964

  # =====================================  第一次循环，计算t_b_m_avg  ==============================
  t_b_m_avg = np.zeros((4,4))
  t_b_m_trans_ls = []
  t_b_m_euler_ls = []
  for fl in fls:
    prefix = os.path.basename(fl).split(".")[0]

    t_b_e = t_b_e_dict[prefix]
    t_c_m = t_c_m_dict[prefix]

    t_b_m = np.dot(np.dot(t_b_e, t_e_c), t_c_m) # 基坐标系->marker（使用当前坐标关系计算得到的值）
    t_b_m_trans = np.array(translation_from_matrix(t_b_m))
    t_b_m_euler = np.array(euler_from_matrix(t_b_m))

    t_b_m_avg += t_b_m

    t_b_m_trans_ls.append(t_b_m_trans)
    t_b_m_euler_ls.append(t_b_m_euler)

  t_b_m_avg = t_b_m_avg / (len(fls))  # 计算t_b_m的均值
  print "[INFO] t_b_m_avg", t_b_m_avg

  t_b_m_trans_ls_alg[alg] = np.array(t_b_m_trans_ls)
  t_b_m_euler_ls_alg[alg] = np.array(t_b_m_euler_ls)
  
  # =======================================  第二次循环，计算重投影  ===============================
  reprojection_error_ls = cal_reprojection_error(t_e_c, t_b_m_avg)

  # real: 0.01+0.001
  # best 
  # error_min: 4.653208213011834

  # sim: 0.01+0.005
  # best [-0.07058943296140564, 0.0006451858029217616, 0.03408238825873804] 
  # error_min: 0.43555603550536476

  # sim: 0.01+0.0025
  # best [-0.07058943296140564, 0.00039518580292176135, 0.034832388258738044] 
  # error_min: 0.3528602827585465

  def t_e_c_optimize(t_e_c, t_b_m_avg, rang=0.01, step=0.00025, mode="xyz"): # (0.01+0.001)-8000点 (0.01+0.005)-64000点
    if alg != algorithm_ls[0]: return # 仅将Tsai作为参考值

    search_range = np.arange(-rang, rang, step) # 搜索区间
    print search_range
    search_range_x = x + search_range if "x" in mode else [x]
    search_range_y = y + search_range if "y" in mode else [y]
    search_range_z = z + search_range if "z" in mode else [z]

    best, error_avg_ls, data_to_save = [], [], []
    error_avg, error_min = 0, 1e9
    cnt, cur_cnt = len(search_range_x)*len(search_range_y)*len(search_range_z), 0
    t_e_c_copy = t_e_c.copy() # 拷贝t_e_c
    for x_ in search_range_x:
      for y_ in search_range_y:
        for z_ in search_range_z:
          # 修改平移参数
          t_e_c_copy[0][3] = x_
          t_e_c_copy[1][3] = y_
          t_e_c_copy[2][3] = z_
          reprojection_error_ls = cal_reprojection_error(t_e_c_copy, t_b_m_avg, log=False)
          error_avg = np.array(reprojection_error_ls).mean()
          error_avg_ls.append(error_avg)
          data_to_save.append([x_, y_, z_, error_avg])

          if cur_cnt % 100 == 0:
            print "[%d/%d]" % (cur_cnt, cnt), [x_, y_, z_], error_avg
          
          if error_avg < error_min:
            error_min = error_avg
            best = [x_, y_, z_]
            print "[DEBUG] error_avg:", error_avg, "\nbest:", best
          
          cur_cnt += 1

    print "[INFO] best", best, "\nerror_min:", error_min
    t_e_c_copy = t_e_c.copy() # 拷贝t_e_c
    t_e_c_copy[0][3] = best[0]
    t_e_c_copy[1][3] = best[1]
    t_e_c_copy[2][3] = best[2]

    # np.savetxt(os.path.join(data_path, "t_e_c_optimized.txt"), t_e_c_copy) # 保存优化后的t_e_c
    # np.savetxt(os.path.join(data_path, "t_e_c_optimized_error.txt"), [error_min]) # 保存优化后的重投影误差

    # np.savetxt(os.path.join(data_path, "t_e_c_optimize.csv"), data_to_save, delimiter=',')
    data_to_save = np.loadtxt(os.path.join(data_path, "t_e_c_optimize.csv"), delimiter=',')

    ''' 可视化xyz对重投影误差的影响'''
    # fig=plt.figure(figsize=(6,5.5))
    # ax = fig.add_subplot(111, projection='3d')
    # ax.view_init(azim=-45, elev=36)

    # cmap = plt.cm.jet.reversed()
    # data_to_save = np.array(data_to_save)
    # # data_to_save = data_to_save[data_to_save[:,1] > 0]
    # ax.scatter(data_to_save[:,0], data_to_save[:,1], data_to_save[:,2], c=data_to_save[:,3], cmap=cmap, s=25, marker="o")

    # ax.xaxis.set_major_locator(plt.MaxNLocator(4)) # 设置刻度数量限制
    # ax.yaxis.set_major_locator(plt.MaxNLocator(4)) # 设置刻度数量限制
    # ax.zaxis.set_major_locator(plt.MaxNLocator(4)) # 设置刻度数量限制
    # # ax.grid(False)
    # ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    # norm = mpl.colors.Normalize(vmin=data_to_save[:,3].min(), vmax=data_to_save[:,3].max())
    # bounds = np.linspace(data_to_save[:,3].min(), data_to_save[:,3].max(), 6)

    # # 顶部显示颜色条
    # # c_map_ax = fig.add_axes([0.26, 0.90, 0.55, 0.04])
    # # color_bar = mpl.colorbar.ColorbarBase(c_map_ax, norm=norm, cmap=cmap, ticks=bounds, orientation = 'horizontal')

    # # 左侧显示颜色条
    # c_map_ax = fig.add_axes([0.15, 0.25, 0.04, 0.55])
    # color_bar = mpl.colorbar.ColorbarBase(c_map_ax, norm=norm, cmap=cmap, ticks=bounds, orientation = 'vertical')
    # c_map_ax.yaxis.set_ticks_position('left')
    # color_bar.set_ticks([0.70,5.0,10.0,15.0,20.0,25.0,30.0])
    # c_map_ax.tick_params(labelsize=12)

    # text_ax = fig.add_axes([0.02, 0.25, 0.04, 0.55])
    # text_ax.text(0, 0.5, "$\mathregular{err\_proj_{avg}}$ (pix)", rotation=90, size=12)
    # text_ax.axis("off")
    
    # # 绘制平面
    # min_idx = np.where(data_to_save[:,3] == np.min(data_to_save[:,3])) # 最小值所在位置
    # # print data_to_save[min_idx][:3] # [-7.05894330e-02  1.45185803e-04  3.55823883e-02  6.98609148e-01]
    # [x_min, y_min, z_min] = data_to_save[min_idx][0][:3]

    # ax.plot([x_min, np.max(data_to_save[:,0])], [y_min, y_min],zs=[z_min, z_min], c='r', lw=2, label="x=%.5f" % x_min)
    # ax.plot([x_min, x_min], [y_min, np.max(data_to_save[:,1])],zs=[z_min, z_min], c='g', lw=2, label="y=%.5f" % y_min)
    # ax.plot([x_min, x_min], [y_min, y_min],zs=[z_min, np.max(data_to_save[:,2])], c='b', lw=2, label="z=%.5f" % z_min)
    # ax.scatter([x_min], [y_min], [z_min], c='darkred', s=25, marker="o", label="e=%.5f" % np.min(data_to_save[:,3]))
    # # ax.text(-0.08, 0.00, 0.04, "(%.4f, %.4f, %.4f)" % (x_min, y_min, z_min), size=11)
    # ax.legend(bbox_to_anchor=(0.62,0.75))

    
    # ax.set_xlabel('X(m)', size=13)
    # ax.set_ylabel('Y(m)', size=13)
    # ax.set_zlabel('Z(m)', size=13)
    # ax.tick_params(labelsize=12)

    # plt.savefig(os.path.join(data_path, "Reprojection error - xyz.svg")) #, bbox_inches='tight')
    # plt.show()

    ''' 可视化单个参数优化对重投影误差的影响 '''
    if len(mode) == 1:
      if mode == "x":
        search_range = search_range_x
        line_pos = x
        color = 'r'
      elif mode == "y":
        search_range = search_range_y
        line_pos = y
        color = 'limegreen'
      elif mode == "z":
        search_range = search_range_z
        line_pos = z
        color = 'deepskyblue'
      
      # plt.figure(figsize=(6, 4.5))
      # plt.plot(search_range, error_avg_ls, "-", color=color)
      # plt.axvline(x=line_pos, color="m")

      # plt.tick_params(labelsize=11)
      # plt.xlabel("%s(m)" % mode.upper(), size=12)
      # plt.ylabel("error", size=12)
      # plt.gca().axes.xaxis.set_major_locator(plt.MaxNLocator(6)) # 设置刻度数量限制

      # plt.savefig(os.path.join(data_path, "Reprojection error - %s.svg" % mode))# , bbox_inches='tight')
      # plt.show()

    return search_range, best, error_min, error_avg_ls

  # 最小化重投影误差，以优化t_e_c
  # start = time.time()
  # best_t_e_c = t_e_c_optimize(t_e_c, t_b_m_avg)
  # print "[DEBUG] took", time.time() - start # 1000000 - 1109.64931393s
  # exit()


  ''' 在同一张图中可视化xyz单独对重投影误差的影响 '''
  # error_avg_ls_x_y_z = []
  # search_range_x_y_z = []
  # best_t_e_c_ls = []
  # error_min_ls = []
  # for ax in ["x", "y", "z"]:
  #   search_range, best_t_e_c, error_min, error_avg_ls = t_e_c_optimize(t_e_c, t_b_m_avg, rang=0.01, step=0.001, mode=ax)
  #   error_avg_ls_x_y_z.append(error_avg_ls)
  #   search_range_x_y_z.append(search_range)
  #   best_t_e_c_ls.append(best_t_e_c)
  #   error_min_ls.append(error_min)
 
  # print "best_t_e_c_ls", best_t_e_c_ls
  # print "error_min_ls", error_min_ls

  # # step=0.001
  # # best_t_e_c_ls [[-0.07058943296140567, -0.0008548141970782484, 0.032582388258738035], [-0.07058943296140566, 0.00014518580292174205, 0.032582388258738035], [-0.07058943296140566, -0.0008548141970782484, 0.03858238825873802]]
  # # error_min_ls [2.9347640693972505, 1.034310949082732, 2.5213531415570083]

  # # step=0.0001
  # # best_t_e_c_ls [[-0.07038943296140572, -0.0008548141970782484, 0.032582388258738035], [-0.07058943296140566, 0.0005451858029216824, 0.032582388258738035], [-0.07058943296140566, -0.0008548141970782484, 0.03848238825873794]]
  # # error_min_ls [2.918026855715374, 0.6990530998371551, 2.5209344214504164]

  # fig = plt.figure(figsize=(8,2.5))
  # plt.subplots_adjust(wspace=0.00)

  # ax1 = plt.subplot(1, 3, 1)
  # ax1.plot(search_range_x_y_z[0], error_avg_ls_x_y_z[0], "-", color='r')
  # # ax1.axvline(x=x, color="k", linestyle='--', lw=0.9, label="x=%.2f" % x)
  # ax1.axvline(x=best_t_e_c_ls[0][0], color="m", linestyle='--', lw=0.9, label="x=%.5f" % best_t_e_c_ls[0][0])
  # ax1.axhline(y=error_min_ls[0], color="k", linestyle='--', lw=0.9, label="e=%.5f" % error_min_ls[0])
  # ax1.legend(loc="upper right", prop = {'size':10})
  
  # ax1.set_xlabel("X(m)", size=12)
  # ax1.set_ylabel("$\mathregular{err\_proj_{avg}}$ (pix)", size=12)
  # ax1.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制

  # ax2 = plt.subplot(1, 3, 2)
  # ax2.plot(search_range_x_y_z[1], error_avg_ls_x_y_z[1], "-", color='limegreen')
  # # ax2.axvline(x=y, color="k", linestyle='--', lw=0.9)
  # ax2.axvline(x=best_t_e_c_ls[1][1], color="m", linestyle='--', lw=0.9, label="y=%.5f" % best_t_e_c_ls[1][1])
  # ax2.axhline(y=error_min_ls[1], color="k", linestyle='--', lw=0.9, label="e=%.5f" % error_min_ls[1])
  # ax2.legend(loc="upper right", prop = {'size':10})
  # ax2.set_xlabel("Y(m)", size=12)
  # ax2.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制

  # ax3 = plt.subplot(1, 3, 3)
  # ax3.plot(search_range_x_y_z[2], error_avg_ls_x_y_z[2], "-", color='deepskyblue')
  # # ax3.axvline(x=z, color="k", linestyle='--', lw=0.9)
  # ax3.axvline(x=best_t_e_c_ls[2][2], color="m", linestyle='--', lw=0.9, label="z=%.5f" % best_t_e_c_ls[2][2])
  # ax3.axhline(y=error_min_ls[2], color="k", linestyle='--', lw=0.9, label="e=%.5f" % error_min_ls[2])
  # ax3.legend(loc="upper right", prop = {'size':10})
  # ax3.set_xlabel("Z(m)", size=12)
  # ax3.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制

  # # plt.tick_params(labelsize=11)
  
  # plt.tight_layout() # 解决label显示超出画面
  # plt.savefig(os.path.join(data_path, "Reprojection error - xyz_split.svg"), bbox_inches='tight')
  # plt.show()
  # exit()

  """ =======================================  1、分析比较重投影误差 ==================================== """
  # 计算平均重投影误差
  reprojection_error_ls = np.array(reprojection_error_ls)[:20]
  reprojection_error_ls_alg.append(reprojection_error_ls)
  error_avg = reprojection_error_ls.mean()
  reprojection_error_alg[alg] = error_avg
  print "[INFO] error_avg", error_avg

  ''' 绘制单个算法重投影误差直方图 '''
  # fig = plt.figure(figsize=(8,6))
  
  # # colors = plt.cm.jet(reprojection_error_ls / reprojection_error_ls.max())
  # # colors = plt.cm.jet(np.linspace(reprojection_error_ls.min() / reprojection_error_ls.max(), 1, len(reprojection_error_ls)))
  
  # plt.barh(range(1, len(reprojection_error_ls)+1), reprojection_error_ls, height=0.7, color='deepskyblue', edgecolor='w', alpha=0.8) # 'steelblue'

  # plt.axvline(x=error_avg, color="r")
  # plt.text(error_avg + 0.02, 10, '<-- Mean (%.3f)' % error_avg, size=11, color="r")

  # for x, y in enumerate(reprojection_error_ls):
  #     plt.text(y + 0.015, x + 1 - 0.25, '%.3f' % y)

  # plt.gca().axes.yaxis.set_major_locator(ticker.MultipleLocator(2)) # 设置刻度密度
  # # plt.ylim(0, len(reprojection_error_ls))
  # plt.xlim(0, max(reprojection_error_ls) + 0.5)
  # plt.yticks([1,5,10,15,20])

  # plt.xlabel("error (pix)", size=12)
  # plt.ylabel("sample", size=12)

  # plt.savefig(os.path.join(data_path, "Reprojection error - %s.svg" % alg), dpi=600, bbox_inches='tight')
  # plt.title(alg)
  # plt.show()


# 打印所有算法的重投影误差
print "\nReprojection_error:\n", reprojection_error_alg
print reprojection_error_ls_alg

''' 绘制各算法重投影误差折线图 '''
fig = plt.figure(figsize=(9,4.5))

colors = ['r', 'limegreen', 'deepskyblue', 'orange', 'c']
marker = "o"
for idx, alg in enumerate(algorithm_ls):
  reprojection_error_ls = reprojection_error_ls_alg[idx]
  if alg == "optimized": marker = "s"
  plt.plot(range(1, len(reprojection_error_ls)+1), reprojection_error_ls, "-.", marker=marker, label=alg[0].upper()+alg[1:], color=colors[idx])
  plt.axhline(y=reprojection_error_alg[alg], color=colors[idx], label="%.5f" % reprojection_error_alg[alg])

if mode == "sim":
  plt.gca().axes.yaxis.set_major_locator(ticker.MultipleLocator(0.5)) # 设置刻度密度
plt.xticks(range(1, len(reprojection_error_ls)+1))

plt.ylabel("err_proj (pix)", size=12)
plt.xlabel("sample", size=12)
plt.legend(loc="upper left", ncol=len(algorithm_ls), prop={'size': 10})

if "optimized" in algorithm_ls:
  plt.savefig(os.path.join(data_path, "Reprojection error - optimized.svg"), dpi=600, bbox_inches='tight')
else:
  plt.savefig(os.path.join(data_path, "Reprojection error.svg"), dpi=600, bbox_inches='tight')

plt.show()

""" ======================================  2、计算t_e_c与真实值的欧式距离 ============================= """
# dis_trans_ls, dis_euler_ls = [], []
# for alg in algorithm_ls:
#   t_e_c = np.loadtxt(os.path.join(data_path, "t_e_c_%s.txt" % alg))
#   # 计算与真实值的差别
#   dis_trans_ls.append(np.linalg.norm(t_e_c_trans_real - translation_from_matrix(t_e_c)))
#   dis_euler_ls.append(np.linalg.norm(t_e_c_euler_real - np.array(euler_from_matrix(t_e_c))))


# # 欧式距离统计图
# plt.figure(figsize=(7, 4))
# width = 0.4
# x = range(len(algorithm_ls))
# rects1 = plt.bar(x=x, height=dis_trans_ls, width=width, alpha=0.8, color='deepskyblue', label="$\mathregular{err_t}$ (m)")
# rects2 = plt.bar(x=[i + width for i in x], height=dis_euler_ls, width=width, color='orange', label="$\mathregular{err_R}$ (rad)")
# plt.ylim(0, 0.006) # y轴取值范围
# plt.xticks([index + width/2 for index in x], algorithm_ls, size=12)
# plt.legend(prop={'size': 12}, ncol=2) # 设置题注
# # 编辑文本
# for rect in rects1:
#     height = rect.get_height()
#     plt.text(rect.get_x() + rect.get_width() / 2, height, "%.4f" % height, ha="center", va="bottom")
# for rect in rects2:
#     height = rect.get_height()
#     plt.text(rect.get_x() + rect.get_width() / 2, height, "%.4f" % height, ha="center", va="bottom")

# if "optimized" in algorithm_ls:
#   plt.savefig(os.path.join(data_path, "Euclidean distance of translation and rotation - optimized.svg"), dpi=600, bbox_inches='tight')
#   np.savetxt(os.path.join(data_path, "Euclidean distance of translation and rotation - optimized.txt"), [dis_trans_ls, dis_euler_ls]) # 保存优化后的实际误差
# else:
#   plt.savefig(os.path.join(data_path, "Euclidean distance of translation and rotation.svg"), dpi=600, bbox_inches='tight')
#   np.savetxt(os.path.join(data_path, "Euclidean distance of translation and rotation.txt"), [dis_trans_ls, dis_euler_ls]) # 保存优化后的实际误差
# plt.show()


""" ======================================  3、比较t_b_m变动(用于仿真数据) ============================= """
# 计算均值及标准差
ptp_trans_ls, ptp_euler_ls = [], []
# for alg in algorithm_ls:
#   print alg
  # print "标准差"
  # print "x", np.std(t_b_m_trans_ls_alg[alg][:,0])
  # print "y", np.std(t_b_m_trans_ls_alg[alg][:,1])
  # print "z", np.std(t_b_m_trans_ls_alg[alg][:,2])
  # print "xyz标准差之和", np.std(t_b_m_trans_ls_alg[alg][:,0]) + np.std(t_b_m_trans_ls_alg[alg][:,1]) + np.std(t_b_m_trans_ls_alg[alg][:,2])

  # print "r", np.std(t_b_m_euler_ls_alg[alg][:,0])
  # print "p", np.std(t_b_m_euler_ls_alg[alg][:,1])
  # print "y", np.std(t_b_m_euler_ls_alg[alg][:,2])
  # print "rpy标准差之和", np.std(t_b_m_euler_ls_alg[alg][:,0]) + np.std(t_b_m_euler_ls_alg[alg][:,1]) + np.std(t_b_m_euler_ls_alg[alg][:,2])


  # print "方差"
  # print "x", np.var(t_b_m_trans_ls_alg[alg][:,0])
  # print "y", np.var(t_b_m_trans_ls_alg[alg][:,1])
  # print "z", np.var(t_b_m_trans_ls_alg[alg][:,2])
  # print "xyz方差之和", np.var(t_b_m_trans_ls_alg[alg][:,0]) + np.var(t_b_m_trans_ls_alg[alg][:,1]) + np.var(t_b_m_trans_ls_alg[alg][:,2])

  # print "r", np.var(t_b_m_euler_ls_alg[alg][:,0])
  # print "p", np.var(t_b_m_euler_ls_alg[alg][:,1])
  # print "y", np.var(t_b_m_euler_ls_alg[alg][:,2])
  # print "rpy方差之和", np.var(t_b_m_euler_ls_alg[alg][:,0]) + np.var(t_b_m_euler_ls_alg[alg][:,1]) + np.var(t_b_m_euler_ls_alg[alg][:,2])


  # print "均值"
  # print "x", np.mean(t_b_m_trans_ls_alg[alg][:,0])
  # print "y", np.mean(t_b_m_trans_ls_alg[alg][:,1])
  # print "z", np.mean(t_b_m_trans_ls_alg[alg][:,2])

  # print "r", np.mean(t_b_m_euler_ls_alg[alg][:,0])
  # print "p", np.mean(t_b_m_euler_ls_alg[alg][:,1])
  # print "y", np.mean(t_b_m_euler_ls_alg[alg][:,2])

  # print "极差"
  # # print "x", np.ptp(t_b_m_trans_ls_alg[alg][:,0])
  # # print "y", np.ptp(t_b_m_trans_ls_alg[alg][:,1])
  # # print "z", np.ptp(t_b_m_trans_ls_alg[alg][:,2])
  # print "xyz极差之和", np.ptp(t_b_m_trans_ls_alg[alg][:,0]) + np.ptp(t_b_m_trans_ls_alg[alg][:,1]) + np.ptp(t_b_m_trans_ls_alg[alg][:,2])
  # ptp_trans_ls.append(np.ptp(t_b_m_trans_ls_alg[alg][:,0]) + np.ptp(t_b_m_trans_ls_alg[alg][:,1]) + np.ptp(t_b_m_trans_ls_alg[alg][:,2]))

  # # print "r", np.ptp(t_b_m_euler_ls_alg[alg][:,0])
  # # print "p", np.ptp(t_b_m_euler_ls_alg[alg][:,1])
  # # print "y", np.ptp(t_b_m_euler_ls_alg[alg][:,2])
  # print "rpy极差之和", np.ptp(t_b_m_euler_ls_alg[alg][:,0]) + np.ptp(t_b_m_euler_ls_alg[alg][:,1]) + np.ptp(t_b_m_euler_ls_alg[alg][:,2])
  # ptp_euler_ls.append(np.ptp(t_b_m_euler_ls_alg[alg][:,0]) + np.ptp(t_b_m_euler_ls_alg[alg][:,1]) + np.ptp(t_b_m_euler_ls_alg[alg][:,2]))


  # print "变异系数"
  # print "x", np.std(t_b_m_trans_ls_alg[alg][:,0]) / np.mean(t_b_m_trans_ls_alg[alg][:,0])
  # print "y", np.std(t_b_m_trans_ls_alg[alg][:,1]) / np.mean(t_b_m_trans_ls_alg[alg][:,1])
  # print "z", np.std(t_b_m_trans_ls_alg[alg][:,2]) / np.mean(t_b_m_trans_ls_alg[alg][:,2])
  # print "xyz变异系数之和", abs(np.std(t_b_m_trans_ls_alg[alg][:,0]) / np.mean(t_b_m_trans_ls_alg[alg][:,0])) + abs(np.std(t_b_m_trans_ls_alg[alg][:,1]) / np.mean(t_b_m_trans_ls_alg[alg][:,1])) + abs(np.std(t_b_m_trans_ls_alg[alg][:,2]) / np.mean(t_b_m_trans_ls_alg[alg][:,2]))

  # print "r", np.std(t_b_m_euler_ls_alg[alg][:,0]) / np.mean(t_b_m_euler_ls_alg[alg][:,0])
  # print "p", np.std(t_b_m_euler_ls_alg[alg][:,1]) / np.mean(t_b_m_euler_ls_alg[alg][:,1])
  # print "y", np.std(t_b_m_euler_ls_alg[alg][:,2]) / np.mean(t_b_m_euler_ls_alg[alg][:,2])
  # print "rpy变异系数之和", abs(np.std(t_b_m_euler_ls_alg[alg][:,0]) / np.mean(t_b_m_euler_ls_alg[alg][:,0])) + abs(np.std(t_b_m_euler_ls_alg[alg][:,1]) / np.mean(t_b_m_euler_ls_alg[alg][:,1])) + abs(np.std(t_b_m_euler_ls_alg[alg][:,2]) / np.mean(t_b_m_euler_ls_alg[alg][:,2]))


# print "协方差", np.cov([ptp_trans_ls, dis_trans_ls], bias=1)
# print "相关系数", np.corrcoef([ptp_trans_ls, dis_trans_ls])

# print "协方差", np.cov([ptp_euler_ls, dis_euler_ls], bias=1)
# print "相关系数", np.corrcoef([ptp_euler_ls, dis_euler_ls])


# 绘制箱型图，反映数据变动范围
# x_ls, y_ls, z_ls = [], [], []
# algorithm_ls = algorithm_ls[::-1] # 颠倒顺序
# for alg in algorithm_ls:
#   x_ls.append(t_b_m_trans_ls_alg[alg][:,0])
#   y_ls.append(t_b_m_trans_ls_alg[alg][:,1])
#   z_ls.append(t_b_m_trans_ls_alg[alg][:,2])

# fig = plt.figure(figsize=(9, 3.5))
# plt.subplots_adjust(wspace=0.025, hspace=0.05) # 设置子图间距

# ax1 = plt.subplot(1,3,1)
# # ax1.xaxis.grid(True)
# # if mode == "sim":
# #   ax1.set_xlabel("X (real: %.3fm)" % t_b_m_trans_real[0], size=12)
# # else:
# ax1.set_xlabel("X (m)", size=11)
# plt.tick_params(labelsize=10)
# ax1.axvline(x=t_b_m_trans_real[0], color="r", linestyle='dashed', linewidth=0.8)
# bplot1 = ax1.boxplot(x_ls, labels=algorithm_ls, vert=False, showmeans=True, meanline=True, patch_artist=True)
# # ax1.xaxis.set_major_locator(ticker.MultipleLocator(0.001)) # 设置刻度密度
# ax1.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制

# ax2 = plt.subplot(1,3,2)
# # ax2.xaxis.grid(True)
# # if mode == "sim":
# #   ax2.set_xlabel("Y (real: %.3fm)" % t_b_m_trans_real[1], size=12)
# # else:
# ax2.set_xlabel("Y (m)", size=11)
# plt.tick_params(labelsize=10)
# ax2.axvline(x=t_b_m_trans_real[1], color="r", linestyle='dashed', linewidth=0.8)
# bplot2 = ax2.boxplot(y_ls, vert=False, showmeans=True, meanline=True, patch_artist=True)
# ax2.set_yticks([]) # 隐藏刻度及数字
# # ax2.xaxis.set_major_locator(ticker.MultipleLocator(0.0005)) # 设置刻度密度
# ax2.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制
# # if mode == "sim":
# #   ax2.set_xticks([-0.002, -0.001, 0])

# ax3 = plt.subplot(1,3,3)
# # ax3.xaxis.grid(True)
# # if mode == "sim":
# #   ax3.set_xlabel("Z (real: %.3fm)" % t_b_m_trans_real[2], size=12)
# # else:
# ax3.set_xlabel("Z (m)", size=11)
# plt.tick_params(labelsize=10)
# ax3.axvline(x=t_b_m_trans_real[2], color="r", linestyle='dashed', linewidth=0.8)
# bplot3 = ax3.boxplot(z_ls, vert=False, showmeans=True, meanline=True, patch_artist=True)
# ax3.set_yticks([]) # 隐藏刻度及数字
# # ax3.xaxis.set_major_locator(ticker.MultipleLocator(0.001)) # 设置刻度密度
# ax3.xaxis.set_major_locator(plt.MaxNLocator(3)) # 设置刻度数量限制

# colors = ['pink', 'lightblue', 'lightgreen', 'deepskyblue', 'cyan']
# for bplot in (bplot1, bplot2, bplot3):
#     for patch, color in zip(bplot['boxes'], colors):
#         patch.set(facecolor=color, linewidth=1)

# plt.savefig(os.path.join(data_path, "t_b_m trans analysis.svg"), dpi=600, bbox_inches='tight')
# plt.show()

# TODO：计算欧氏距离与重投影误差的协方差







""" ===========================================  实验记录 ====================================== 
标定钉顶点对准标定板中心位置：
ee_link位置：(53.514cm, 0.406cm, 32.409cm)，末端垂直于桌面
标定钉长度：7.00cm
可得标定板中心位置：(53.514cm, 0.406cm, 25.409cm)



"""