#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
控制机械臂移动到多个姿态，拍摄二维码图片并记录TF数据，最后通过easy_handeye求解手眼转换矩阵
'''

import time
import rospy, sys, tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, PointStamped
from copy import deepcopy
from math import pi
from tf.transformations import *
from transforms3d import quaternions

import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# from easy_handeye.handeye_client import HandeyeClient


import rospy
import std_srvs
from std_srvs import srv
import easy_handeye as hec
from easy_handeye.handeye_calibration import HandeyeCalibrationParameters
import easy_handeye_msgs as ehm
from easy_handeye_msgs import srv

path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))  # 上级目录


# def trans_quat_to_mat44(trans, quat):
#   # 平移及四元数数组转齐次变换矩阵
#   trans = translation_matrix(trans)
#   mat44 = quaternion_matrix(quat)
#   mat44[:, 3] = trans[:, 3]
#   return mat44

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


# 验证，表示为齐次变换矩阵时，t_b_m为t_m_b的逆矩阵
# trans, quat = self.lookup_transform("base_link", "aruco_marker") # t_b_m
# print "trans, quat1", trans, quat
# t_b_m = trans_quat_to_mat44(trans, quat)
# print "t_b_m", t_b_m

# trans, quat = mat44_to_trans_quat(t_b_m)
# print "trans, quat2", trans, quat

# trans, quat = self.lookup_transform("aruco_marker", "base_link") # t_m_b
# t_m_b = trans_quat_to_mat44(trans, quat)
# print "t_m_b", t_m_b
# print "t_m_b.inv", np.linalg.inv(t_m_b)

# print numpy.allclose(t_b_m, np.linalg.inv(t_m_b))


# 修改自~/catkin_ws/src/easy_handeye/easy_handeye/src/easy_handeye/handeye_client.py
# 原文件中的命名空间处理存在一点问题，set_namespace函数中的ros service未添加命名空间(必须在运行脚本时指定正确的命名空间)，这里手动添加
# 即将hec.GET_SAMPLE_LIST_TOPIC 修改为 namespace+hec.GET_SAMPLE_LIST_TOPIC
class HandeyeClient(object):
  def __init__(self, namespace=None):
    if namespace is None:
        namespace = rospy.get_namespace()

    if not namespace.endswith('/'):
            namespace = namespace+'/'

    self.parameters = None
    self.list_algorithms_proxy = None
    self.set_algorithm_proxy = None
    self.get_sample_proxy = None
    self.take_sample_proxy = None
    self.remove_sample_proxy = None
    self.compute_calibration_proxy = None
    self.save_calibration_proxy = None
    self.check_starting_pose_proxy = None
    self.enumerate_target_poses_proxy = None
    self.select_target_pose_proxy = None
    self.plan_to_selected_target_pose_proxy = None
    self.execute_plan_proxy = None

    if namespace != "/":
        rospy.loginfo("Configuring for calibration with namespace: {}".format(namespace))
        self.set_namespace(namespace)
    else:
        rospy.logwarn("No namespace was passed at construction; remember to use set_namespace()")

  def set_namespace(self, namespace):
    self.parameters = HandeyeCalibrationParameters.init_from_parameter_server(namespace)

    # init services: sampling
    rospy.wait_for_service(namespace+hec.GET_SAMPLE_LIST_TOPIC)
    self.get_sample_proxy = rospy.ServiceProxy(namespace+hec.GET_SAMPLE_LIST_TOPIC, ehm.srv.TakeSample)
    rospy.wait_for_service(namespace+hec.TAKE_SAMPLE_TOPIC)
    self.take_sample_proxy = rospy.ServiceProxy(namespace+hec.TAKE_SAMPLE_TOPIC, ehm.srv.TakeSample)
    rospy.wait_for_service(namespace+hec.REMOVE_SAMPLE_TOPIC)
    self.remove_sample_proxy = rospy.ServiceProxy(namespace+hec.REMOVE_SAMPLE_TOPIC, ehm.srv.RemoveSample)

    # init services: calibration
    rospy.wait_for_service(namespace+hec.LIST_ALGORITHMS_TOPIC)
    self.list_algorithms_proxy = rospy.ServiceProxy(namespace+hec.LIST_ALGORITHMS_TOPIC, ehm.srv.ListAlgorithms)
    rospy.wait_for_service(namespace+hec.SET_ALGORITHM_TOPIC)
    self.set_algorithm_proxy = rospy.ServiceProxy(namespace+hec.SET_ALGORITHM_TOPIC, ehm.srv.SetAlgorithm)
    rospy.wait_for_service(namespace+hec.COMPUTE_CALIBRATION_TOPIC)
    self.compute_calibration_proxy = rospy.ServiceProxy(namespace+hec.COMPUTE_CALIBRATION_TOPIC, ehm.srv.ComputeCalibration)
    rospy.wait_for_service(namespace+hec.SAVE_CALIBRATION_TOPIC)
    self.save_calibration_proxy = rospy.ServiceProxy(namespace+hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

    # init services: robot movement
    rospy.wait_for_service(namespace+hec.CHECK_STARTING_POSE_TOPIC)
    self.check_starting_pose_proxy = rospy.ServiceProxy(namespace+hec.CHECK_STARTING_POSE_TOPIC, ehm.srv.CheckStartingPose)
    rospy.wait_for_service(namespace+hec.ENUMERATE_TARGET_POSES_TOPIC)
    self.enumerate_target_poses_proxy = rospy.ServiceProxy(namespace+hec.ENUMERATE_TARGET_POSES_TOPIC, ehm.srv.EnumerateTargetPoses)
    rospy.wait_for_service(namespace+hec.SELECT_TARGET_POSE_TOPIC)
    self.select_target_pose_proxy = rospy.ServiceProxy(namespace+hec.SELECT_TARGET_POSE_TOPIC, ehm.srv.SelectTargetPose)
    rospy.wait_for_service(namespace+hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC)
    self.plan_to_selected_target_pose_proxy = rospy.ServiceProxy(namespace+hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC, ehm.srv.PlanToSelectedTargetPose)
    rospy.wait_for_service(namespace+hec.EXECUTE_PLAN_TOPIC)
    self.execute_plan_proxy = rospy.ServiceProxy(namespace+hec.EXECUTE_PLAN_TOPIC, ehm.srv.ExecutePlan)

  # services: sampling

  def get_sample_list(self):
    return self.get_sample_proxy().samples

  def take_sample(self):
    return self.take_sample_proxy().samples

  def remove_sample(self, index):
    return self.remove_sample_proxy(ehm.srv.RemoveSampleRequest(sample_index=index)).samples

  def list_algorithms(self):
    return self.list_algorithms_proxy()

  def set_algorithm(self, new_algorithm):
    return self.set_algorithm_proxy(new_algorithm)

  def compute_calibration(self):
    return self.compute_calibration_proxy()

  def save(self):
    return self.save_calibration_proxy()


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
  def __init__(self, handeye_client, mode):
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化ROS节点
    rospy.init_node('demo', anonymous=True)
                    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('arm')
    robot = moveit_commander.RobotCommander()
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(False)
    
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
            
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.0001)
    
    if mode == "sim":
      arm.set_max_velocity_scaling_factor(1.0)
    elif mode == "real":
      arm.set_max_velocity_scaling_factor(0.5)

    arm.set_planning_time(0.05) # 规划时间限制
    
    # 获取终端link的名称
    eef_link = arm.get_end_effector_link()

    # print "[INFO] Current pose:", arm.get_current_pose(eef_link).pose    

    self.group = arm
    self.robot = robot
    self.eef_link = eef_link
    self.reference_frame = reference_frame
    self.moveit_commander = moveit_commander

    # 控制机械臂运动到之前设置的姿态
    # arm.set_named_target('calib1')
    # arm.go()

    # self.go_to_joint_state([2.5228, 0.8964, 0.9746, 0.8614, 2.7515, -1.8821, 0.1712])

    self.mode = mode
    self.listener = tf.TransformListener()
    self.handeye_client = handeye_client
    self.im_converter = image_converter()
    self.aruco_pix_listener = aruco_pix_listener()
    self.data_num = 0

    if mode == "sim":
      self.camera_link = "camera_link_optical"
      self.states = [
        [2.5230, 0.8961, 0.9750, 0.8609, 2.7510, -1.8819, 0.1710],
        [2.4861, 0.9171, 0.9988, 0.8576, 2.8072, -1.8743, 0.1717],
        [2.5231, 0.8961, 0.9738, 0.8604, 2.7517, -1.8818, 0.1700],
        [2.4003, 0.7911, 0.9820, 1.0503, 2.8352, -1.6524, 0.3760],
        [2.5049, 0.9006, 0.9405, 0.8542, 2.8436, -1.8724, 0.4127],
        [2.6273, 0.8107, 0.9139, 0.9618, 2.7511, -1.7930, 0.5062],
        [2.7162, 0.8069, 0.8263, 0.9200, 2.7147, -1.8470, 0.7433],
        [2.7065, 0.8050, 0.8584, 0.9477, 2.4444, -1.8806, 0.1071],
        [2.2901, 0.7806, 1.0289, 1.2211, 2.7996, -1.7092, 0.1444],
        [2.1136, 0.7398, 1.0706, 1.2892, 2.7842, -1.5650, -0.0094],
        [2.7180, 0.7731, 1.0472, 1.2663, 2.4937, -1.7672, 0.5462],
        [2.7180, 0.7731, 1.0472, 1.2663, 2.4937, -1.7672, 0.5462],
        [2.6515, 0.6861, 1.0594, 1.4848, 2.5607, -1.5177, 0.8066],
        [2.7958, 0.8070, 0.9759, 1.2940, 2.4786, -1.6845, 0.7778],
        [2.8772, 0.7243, 0.9153, 1.4943, 2.3478, -1.4724, 1.0759],
        [2.3152, 0.8412, 1.1005, 1.1974, 2.5480, -1.6883, 0.2808],
        [2.0790, 0.8522, 1.0670, 1.6003, 2.6245, -1.2628, 0.5669],
        [2.1475, 1.0192, 1.0940, 1.3640, 2.6444, -1.3627, 0.3612],

        [0.2200, -0.5008, -2.8969, 1.7277, 2.6600, -1.1654, 1.5059],
        [0.4877, -0.7181, -0.2344, -1.3239, -0.1559, -1.4678, 0.5835],
        [0.3773, -0.6314, -0.2302, -1.4444, -0.1600, -1.3351, 0.4067],
        [-2.2371, 1.3616, 1.5669, -1.2129, 0.9923, -1.8344, -0.1351],
        [-2.3273, 1.3885, 1.5713, -0.9358, 1.0748, -1.9300, 0.5635],
        [-2.2465, 1.2267, 1.8483, -1.0450, 0.7416, -1.6672, 0.3275],
        [-1.7879, 1.2150, 1.6576, -1.5896, 0.7221, -1.4276, 0.6940],
        [-2.2530, 1.0939, 1.8129, -1.3149, 0.7196, -1.6466, 0.4342],
        [-2.1126, 1.1457, 1.7960, -1.2610, 0.6560, -1.6326, 0.4547],
        [-2.1270, 1.2311, 1.8561, -1.0124, 0.5739, -1.8181, 0.4290],
      ]
    elif mode == "real":
      self.camera_link = "camera_color_optical_frame"
      self.states = [
        [-0.1629, -0.1209, -0.0899, -0.6650, 0.1250, -1.9180, -1.7391],
        [0.1768, -0.2480, -0.0637, -0.6867, -0.2229, -1.8912, -1.2967],
        [0.1064, -0.0347, -0.0530, -1.1423, -0.0809, -1.5052, -1.6230],
        [0.4235, -0.0861, -0.1177, -1.1818, -0.2675, -1.4303, -1.5148],
        [0.4016, -0.0026, -0.0992, -1.2851, -0.3538, -1.4122, -1.5428],
        [0.2777, -0.0294, -0.1911, -1.2395, -0.0141, -1.3563, -2.0564],
        [2.9142, 0.1833, -2.7987, -1.0661, -0.0965, -1.4191, -1.8634],
        [2.8310, -0.1007, -2.8406, -1.3918, 0.0263, -1.2798, -1.8718],
        [2.6899, -0.0448, -2.9686, -1.3379, 0.2609, -1.3600, -2.1981],
        [2.7554, -0.2997, -2.5511, -1.5645, -0.2672, -1.2493, -1.5705],
        [2.7252, 0.2366, -2.6045, -0.9946, -0.2552, -1.5880, -1.2153],
        [2.3309, 0.2733, -2.6384, -1.1497, 0.2391, -1.3979, -1.6642],
        [2.1696, 0.5283, -2.6540, -0.8638, 0.3913, -1.5963, -1.8724],
        [2.2318, 0.5134, -2.7120, -0.8413, 0.3381, -1.6116, -1.6351],
        [2.3469, 0.5802, -2.7714, -0.6768, 0.2250, -1.7614, -1.4197],
        [2.3452, 0.4840, -2.7232, -0.8959, 0.2437, -1.5798, -1.5464],
        [2.3888, 0.3196, -2.7301, -1.1076, 0.1213, -1.4436, -1.5656],
        [2.5365, 0.7079, -2.6854, -0.5944, 0.1202, -1.7089, -1.5256],
        [2.3243, 0.5048, -2.5923, -1.0267, 0.1985, -1.3861, -1.6988],
        [2.6772, 0.3100, -2.6433, -1.0222, -0.1752, -1.4295, -1.1358],
        [2.5951, 0.3157, -2.5408, -1.0630, -0.2109, -1.3997, -1.1196],
        [2.5800, 0.3364, -2.4705, -1.0478, -0.2944, -1.4289, -1.6568],
        [2.7948, 0.3616, -2.5705, -1.0294, -0.4474, -1.5064, -0.9428],
        [2.7595, 0.2951, -2.5461, -1.1454, -0.4809, -1.3790, -1.1017],
      ]


  def lookup_transform(self, start, end):
    try:
      self.listener.waitForTransform(start, end, rospy.Time.now(), rospy.Duration(0.5)) # 从现在开始0.5s内找不到转换关系，说明此刻转换未发布
      (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time(0))
      return (cam_trans, cam_quat)
    except Exception as e:
      pass

    # # while not rospy.is_shutdown():
    # for i in range(10000):  # 若转换关系确实存在，10000次尝试以内应能查询到结果
    #   try:
    #     # (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time(0))
    #     (cam_trans, cam_quat) = self.listener.lookupTransform(start, end, rospy.Time.now())
    #     return (cam_trans, cam_quat)
    #   # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #   except Exception as e:
    #     # print e
    #     # print str(Exception)
    #     pass
    
    return (None, None)

  def go_to_joint_state(self, joint_goal):
    group = self.group
    group.go(joint_goal, wait=True)

  def display_sample_list(self, sample_list):
    for i in range(len(sample_list.hand_world_samples)):
        print('{}) \n hand->world {} \n camera->marker {}\n'.format(i,
                                    sample_list.hand_world_samples[i],
                                    sample_list.camera_marker_samples[i]))

  def take_sample(self):
    ret = self.lookup_transform("base_link", "aruco_marker")
    # print "ret", ret
    if ret != (None, None): # 转换关系存在，即识别到了aruco_marker
      # 识别到maker再采集数据，否则会因tf转换失败而退出
      self.handeye_client.take_sample()
      self.display_sample_list(self.handeye_client.get_sample_list())

      # 保存原始图像
      color_path = path + "/data/%s/%06d.color.jpg" % (self.mode, self.data_num)
      cv2.imwrite(color_path, self.im_converter.cv_image_color, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # 保存坐标系可视化的图像
      aruco_path = path + "/data/%s/%06d.aruco.jpg" % (self.mode, self.data_num)
      cv2.imwrite(aruco_path, self.im_converter.cv_image_aruco, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # 保存marker中心点像素坐标
      pix_path = path + "/data/%s/%06d.pixel.txt" % (self.mode, self.data_num)
      np.savetxt(pix_path, self.aruco_pix_listener.aruco_pix)

      # 保存转换关系，保存为齐次变换矩阵， t_c_m表示从相机到标记的转换矩阵，即标记坐标系相对于相机坐标系的转换矩阵
      trans, quat = self.lookup_transform("base_link", "ee_link") # t_b_e
      t_b_e = trans_quat_to_mat44(trans, quat)
      trans, quat = self.lookup_transform(self.camera_link, "aruco_marker") # t_c_m
      t_c_m = trans_quat_to_mat44(trans, quat)

      t_b_e_path = path + "/data/%s/%06d.t_b_e.txt" % (self.mode, self.data_num)
      np.savetxt(t_b_e_path, t_b_e)

      t_c_m_path = path + "/data/%s/%06d.t_c_m.txt" % (self.mode, self.data_num)
      np.savetxt(t_c_m_path, t_c_m)

      self.data_num += 1

  
  def compute_calibration(self, algorithm='OpenCV/Tsai-Lenz'):
    """ AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }"""
    self.handeye_client.set_algorithm(algorithm)

    result = self.handeye_client.compute_calibration()
    translation = result.calibration.transform.transform.translation
    rotation = result.calibration.transform.transform.rotation

    trans = [translation.x, translation.y, translation.z]
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]

    # print "[DEBUG]", trans, quat

    return trans, quat

  
  def auto_move_and_sample(self):
    for state in self.states:
      self.go_to_joint_state(state)
      rospy.sleep(1.0) # 延时以确保tf已更新 0.1s太短，会导致标定误差很大
      self.take_sample()
      rospy.sleep(0.1)


if __name__ == "__main__":
  mode = "real" # ["sim", "real"]

  # 初始化easy_handeye手眼标定客户端
  if mode == "sim":
    namespace = "/mantra_gazebo_eye_on_hand"  # 需根据手眼标定的配置来设置
  elif mode == "real":
    namespace = "/mantra_eye_on_hand"  # 需根据手眼标定的配置来设置

  handeye_client = HandeyeClient(namespace)

  if handeye_client.parameters.eye_on_hand:
      print('eye-on-hand calibration')
      print('robot effector frame: {}'.format(handeye_client.parameters.robot_effector_frame))
  else:
      print('eye-on-base calibration')
      print('robot base frame: {}'.format(handeye_client.parameters.robot_base_frame))
  print('tracking base frame: {}'.format(handeye_client.parameters.tracking_base_frame))
  print('tracking target frame: {}'.format(handeye_client.parameters.tracking_marker_frame))

  # 确保运行标定时，样本数为0
  sample_num = len(handeye_client.get_sample_list().hand_world_samples)
  # print sample_num
  if sample_num > 0:
    for i in range(sample_num-1,-1,-1): # 倒序删除
      # print "remove", i
      rospy.sleep(0.1)
      handeye_client.remove_sample(i)

  # print "result", len(handeye_client.get_sample_list().hand_world_samples)
  # exit()

  try:
    mantra_pickup = MantraPickup(handeye_client, mode)

    # 测试
    # mantra_pickup.take_sample()
    # exit()

    # 采集样本
    mantra_pickup.auto_move_and_sample()

    # 计算和保存标定结果
    algorithm_ls = ['Tsai-Lenz','Park','Horaud','Andreff','Daniilidis']
    for algo in algorithm_ls[:]:
      try: # 仅保存计算成功的(有些算法计算出来的结果会有nan)
        trans, quat = mantra_pickup.compute_calibration('OpenCV/' + algo)
        t_e_c = trans_quat_to_mat44(trans, quat)
        np.savetxt(path + "/data/%s/t_e_c_%s.txt" % (mode, algo), t_e_c)
        print "\nresult of %s:" % algo, trans, quat
      except:
        pass

    # 保存相机内参
    camera_info_path = path + "/data/%s/camera_info.txt" % mode
    mantra_pickup.im_converter.save_camera_info(camera_info_path)

    # 关闭并退出moveit
    mantra_pickup.moveit_commander.roscpp_shutdown()
    mantra_pickup.moveit_commander.os._exit(0)

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    pass



# sim
# result of Tsai-Lenz: [-0.07160164560846347, -0.0011253596536365436, 0.031513237490110285] [-0.0004284022387325089, 0.0022777350290372755, -0.7063960978547398, 0.707812956458312]
# result of Park: [-0.07160145637172599, -0.0011667813296825577, 0.03152371428276636] [-0.000177749527278495, 0.0020999935784514756, -0.7064286616916676, 0.7077811133200683]
# result of Horaud: [-0.07160128768157017, -0.0011616251967257445, 0.031521509277004665] [-0.0001895128150157786, 0.002120060211031727, -0.7064282271194045, 0.7077814841874082]
# result of Andreff: [-0.07205708963826246, -0.00032957470631941876, 0.034356712114669785] [-0.00025540544303507103, 0.0020301570432484928, -0.706446608980315, 0.70776337987399]
# result of Daniilidis: [-0.07198965258711985, -0.0011606763930636893, 0.03156098228350543] [-0.0005500575954816343, 0.002079122054706506, -0.7065332113392672, 0.7076766182111326]

# real
# result of Tsai-Lenz: [-0.03740342649033798, -0.09599532453344377, 0.04845608621993405] [-0.011698604743764584, -0.007038873461284512, 0.006652591997545149, 0.9998846633123039]
# result of Park: [-0.03740229916010351, -0.09599779074857925, 0.048455136641167365] [-0.011700064968230894, -0.007060663088721109, 0.006659844007164881, 0.9998844443205862]
# result of Horaud: [-0.03740500607047, -0.09599622384833523, 0.04845643638058801] [-0.011701226063163725, -0.007037538974864588, 0.0066599332347004525, 0.9998845931621838]
# result of Andreff: [-0.03658663253078889, -0.0980852404258371, 0.06191279892756074] [-0.011583215300019857, -0.007248638702478494, 0.006378193646590222, 0.999886296038745]
# result of Daniilidis: [-0.03704683505236099, -0.09580365954504572, 0.04834402872231559] [-0.01161113589377653, -0.007419552157485243, 0.005984893670170516, 0.9998871500408412]
