#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: mantra mantra hmi implementation use python2 and PyQt5
# Date       : 09/09/2019 3:07 PM
# File Name  : mantra_hmi_pro.py

from __future__ import print_function
import os
import sys
import time

curr_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(curr_path))

from math import pi
# ROS相关
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray, Bool
from sensor_msgs.msg import JointState

# Moveit相关
from mantra_move_client import *

from mantra_hmi_pro_ui import *
from save_states import create_group_state

# QT相关
from PyQt5.Qt import QMutex, pyqtSignal
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QDesktopWidget
from PyQt5.QtGui import QIcon

"""
python中内置数据类型（int、list、dict）的操作都是原子的，多线程读写不需要加锁
"""
# 互斥锁
command_arr_mutex = QMutex()

# 发送给Mantra_driver的控制指令
command_arr = Int32MultiArray()  # ROS中自定义的数据类型，保险起见，对其读写加锁
command_cnt = 4
command_arr.data = [0]*command_cnt  # 0:使能 1:复位 2:置零 3:急停, PubThread及主线程按钮函数均会写，须加锁

joint_ctl_arr = [0]*7 # 关节控制标志位, 可弃用

vel_scaling = 0.0  # 速度调整比例
movej_rad_deg_flag = 1  # 角度显示单位切换标志, 默认为角度
movel_rad_deg_flag = 1  # 角度显示单位切换标志, 默认为角度
movel_m_cm_flag = 1  # 距离显示单位切换标志, 默认为米
curr_joints = [0.0]*7  # 当前关节角
goal_joints = [0.0]*7  # 目标关节角
curr_pose = [0.0]*7  # 当前位置
movel_axis = None  # MoveL 移动的参考轴
movel_value = None  # MoveL 移动的距离/角度

moveJ = False  # 关节运动标志
moveL = False  # 线性运动标志
back_home_flag = False  # 回零点标志
change_vel = False  # 调整速度标志


class PubThread(QtCore.QThread):
    def __init__(self):
        super(PubThread, self).__init__()
        self.pub = rospy.Publisher('mantra_hmi', Int32MultiArray, queue_size=1)  # 发布给Mantra_driver的控制指令

    def run(self):
        global command_cnt
        r = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            # 加锁, 发送/置位command_arr过程中不允许其他线程修改
            command_arr_mutex.lock()

            try:
                self.pub.publish(command_arr)
            except rospy.exceptions.ROSException:
                print ("Stop publish.")
                break
            # print(command_arr.data)

            # 消息已发送, 置位
            command_arr.data = [0]*command_cnt

            command_arr_mutex.unlock()

            r.sleep()

    def stop(self):
        self.terminate()


class SubThread(QtCore.QThread):
    def __init__(self):
        super(SubThread, self).__init__()
        self.sub = rospy.Subscriber("joint_states", JointState, self.callback)
        self.listener = tf.TransformListener()

    def run(self):
        global curr_pose
        r = rospy.Rate(10)  # 10hz
        base_link, ee_link = get_base_ee_link()  # 获取末端坐标系名称
        while not rospy.is_shutdown() and ee_link:
            try:
                (position, orientation) = self.listener.lookupTransform(base_link, ee_link, rospy.Time(0))
                xyz = position
                rpy = list(euler_from_quaternion(orientation))
                curr_pose = xyz + rpy  # 实时获取当前位置
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        r.sleep()

    # 获取关节当前位置
    @staticmethod
    def callback(joint_states):
        global curr_joints
        curr_joints = joint_states.position[:7]  # 获取新状态

    def stop(self):
        self.terminate()


class MoveThread(QtCore.QThread):
    """
    运动控制线程
    """
    back_home_signal = pyqtSignal()  # 此信号用于通知主窗口实例，回零函数是否执行完成

    def __init__(self):
        super(MoveThread, self).__init__()
        if test_all_service():  # 测试运动服务器是否已启动
            print("\033[1;32m%s\033[0m" % "[INFO] You can move the robot after press the power on button now!")
        else:
            exit("[ERROR] Please start the move service!")

    def run(self):
        global moveJ, moveL, back_home_flag, change_vel
        r = rospy.Rate(50)  # 50hz
        # 运动控制指令发送
        while not rospy.is_shutdown():
            if moveJ:  # 关节运动
                move_to_joint_states(goal_joints)
                print("[INFO] Go to joint state...")
                moveJ = False  # 标志复位

            if moveL:  # 线性运动
                move_to_pose_shift(movel_axis, movel_value)
                print("[INFO] Go to pose shift...")
                moveL = False  # 标志复位

            if back_home_flag:  # 回零点
                move_to_pose_named('home')
                # 回到零点函数执行完成，无论返回成功与否, 发信号使能回back_home按钮
                self.back_home_signal.emit()
                back_home_flag = False  # 标志复位

            if change_vel:  # 调整速度
                set_vel_scaling(vel_scaling)
                print ("[INFO] Change speed...")
                change_vel = False  # 标志复位

            r.sleep()

    def stop(self):
        self.terminate()


# FIXME 调试完成后可删除！！！
class WindowThread(QtCore.QThread):
    def __init__(self, window_):
        super(WindowThread, self).__init__()
        self.window = window_

    def run(self):
        global movej_rad_deg_flag
        r = rospy.Rate(2)  # 2hz
        time.sleep(1)  # 休眠一秒等待界面初始化

        while not rospy.is_shutdown():
            # print(curr_joints)
            # 关节角刷新显示
            if movej_rad_deg_flag is 0:
                if curr_joints[0] < 0:
                    self.window.label_1.setText("Joint1 (%.3f rad )" % float(curr_joints[0]))
                else:
                    self.window.label_1.setText("Joint1 ( %.3f rad )" % float(curr_joints[0]))
                if curr_joints[1] < 0:
                    self.window.label_2.setText("Joint2 (%.3f rad )" % float(curr_joints[1]))
                else:
                    self.window.label_2.setText("Joint2 ( %.3f rad )" % float(curr_joints[1]))
                if curr_joints[2] < 0:
                    self.window.label_3.setText("Joint3 (%.3f rad )" % float(curr_joints[2]))
                else:
                    self.window.label_3.setText("Joint3 ( %.3f rad )" % float(curr_joints[2]))
                if curr_joints[3] < 0:
                    self.window.label_4.setText("Joint4 (%.3f rad )" % float(curr_joints[3]))
                else:
                    self.window.label_4.setText("Joint4 ( %.3f rad )" % float(curr_joints[3]))
                if curr_joints[4] < 0:
                    self.window.label_5.setText("Joint5 (%.3f rad )" % float(curr_joints[4]))
                else:
                    self.window.label_5.setText("Joint5 ( %.3f rad )" % float(curr_joints[4]))
                if curr_joints[5] < 0:
                    self.window.label_6.setText("Joint6 (%.3f rad )" % float(curr_joints[5]))
                else:
                    self.window.label_6.setText("Joint6 ( %.3f rad )" % float(curr_joints[5]))
                if curr_joints[6] < 0:
                    self.window.label_7.setText("Joint7 (%.3f rad )" % float(curr_joints[6]))
                else:
                    self.window.label_7.setText("Joint7 ( %.3f rad )" % float(curr_joints[6]))

            else:
                curr_joints_deg = list(curr_joints)
                for i in range(7):  # 弧度转换为度
                    curr_joints_deg[i] = round(float(curr_joints_deg[i] / pi * 180.0), 2)

                if curr_joints_deg[0] < 0:
                    if curr_joints_deg[0] <= -100:
                        self.window.label_1.setText("Joint1 ({:.2f} deg )".format(curr_joints_deg[0]))
                    else:
                        self.window.label_1.setText("Joint1 ({:.3f} deg )".format(curr_joints_deg[0]))
                else:
                    if curr_joints_deg[0] >= 100:
                        self.window.label_1.setText("Joint1 ( {:.2f} deg )".format(curr_joints_deg[0]))
                    else:
                        self.window.label_1.setText("Joint1 ( {:.3f} deg )".format(curr_joints_deg[0]))
                if curr_joints_deg[1] < 0:
                    if curr_joints_deg[1] <= -100:
                        self.window.label_2.setText("Joint2 ({:.2f} deg )".format(curr_joints_deg[1]))
                    else:
                        self.window.label_2.setText("Joint2 ({:.3f} deg )".format(curr_joints_deg[1]))
                else:
                    if curr_joints_deg[1] >= 100:
                        self.window.label_2.setText("Joint2 ( {:.2f} deg )".format(curr_joints_deg[1]))
                    else:
                        self.window.label_2.setText("Joint2 ( {:.3f} deg )".format(curr_joints_deg[1]))
                if curr_joints_deg[2] < 0:
                    if curr_joints_deg[2] <= -100:
                        self.window.label_3.setText("Joint3 ({:.2f} deg )".format(curr_joints_deg[2]))
                    else:
                        self.window.label_3.setText("Joint3 ({:.3f} deg )".format(curr_joints_deg[2]))
                else:
                    if curr_joints_deg[2] >= 100:
                        self.window.label_3.setText("Joint3 ( {:.2f} deg )".format(curr_joints_deg[2]))
                    else:
                        self.window.label_3.setText("Joint3 ( {:.3f} deg )".format(curr_joints_deg[2]))
                if curr_joints_deg[3] < 0:
                    if curr_joints_deg[3] <= -100:
                        self.window.label_4.setText("Joint4 ({:.2f} deg )".format(curr_joints_deg[3]))
                    else:
                        self.window.label_4.setText("Joint4 ({:.3f} deg )".format(curr_joints_deg[3]))
                else:
                    if curr_joints_deg[3] >= 100:
                        self.window.label_4.setText("Joint4 ( {:.2f} deg )".format(curr_joints_deg[3]))
                    else:
                        self.window.label_4.setText("Joint4 ( {:.3f} deg )".format(curr_joints_deg[3]))
                if curr_joints_deg[4] < 0:
                    if curr_joints_deg[4] <= -100:
                        self.window.label_5.setText("Joint5 ({:.2f} deg )".format(curr_joints_deg[4]))
                    else:
                        self.window.label_5.setText("Joint5 ({:.3f} deg )".format(curr_joints_deg[4]))
                else:
                    if curr_joints_deg[4] >= 100:
                        self.window.label_5.setText("Joint5 ( {:.2f} deg )".format(curr_joints_deg[4]))
                    else:
                        self.window.label_5.setText("Joint5 ( {:.3f} deg )".format(curr_joints_deg[4]))
                if curr_joints_deg[5] < 0:
                    if curr_joints_deg[5] <= -100:
                        self.window.label_6.setText("Joint6 ({:.2f} deg )".format(curr_joints_deg[5]))
                    else:
                        self.window.label_6.setText("Joint6 ({:.3f} deg )".format(curr_joints_deg[5]))
                else:
                    if curr_joints_deg[5] >= 100:
                        self.window.label_6.setText("Joint6 ( {:.2f} deg )".format(curr_joints_deg[5]))
                    else:
                        self.window.label_6.setText("Joint6 ( {:.3f} deg )".format(curr_joints_deg[5]))
                if curr_joints_deg[6] < 0:
                    if curr_joints_deg[6] <= -100:
                        self.window.label_7.setText("Joint7 ({:.2f} deg )".format(curr_joints_deg[6]))
                    else:
                        self.window.label_7.setText("Joint7 ({:.3f} deg )".format(curr_joints_deg[6]))
                else:
                    if curr_joints_deg[6] >= 100:
                        self.window.label_7.setText("Joint7 ( {:.2f} deg )".format(curr_joints_deg[6]))
                    else:
                        self.window.label_7.setText("Joint7 ( {:.3f} deg )".format(curr_joints_deg[6]))

                # if curr_joints[0] < 0:
                #     if float(curr_joints[0] / pi * 180.0) <= -100:
                #         self.window.label_1.setText("Joint1 (deg )")
                #     else:
                #         self.window.label_1.setText("Joint1 (deg )")
                # else:
                #     if float(curr_joints[0] / pi * 180.0) >= 100:
                #         self.window.label_1.setText("Joint1 ( deg )")
                #     else:
                #         self.window.label_1.setText("Joint1 (deg )")
                # if curr_joints[1] < 0:
                #     if float(curr_joints[1] / pi * 180.0) <= -100:
                #         self.window.label_2.setText("Joint2 (deg )")
                #     else:
                #         self.window.label_2.setText("Joint2 (deg )")
                # else:
                #     if float(curr_joints[1] / pi * 180.0) >= 100:
                #         self.window.label_2.setText("Joint2 (deg )")
                #     else:
                #         self.window.label_2.setText("Joint2 (deg )")
                # if curr_joints[2] < 0:
                #     if float(curr_joints[2] / pi * 180.0) <= -100:
                #         self.window.label_3.setText("Joint3 ( deg )")
                #     else:
                #         self.window.label_3.setText("Joint3 (deg )")
                # else:
                #     if float(curr_joints[2] / pi * 180.0) >= 100:
                #         self.window.label_3.setText("Joint3 (deg )")
                #     else:
                #         self.window.label_3.setText("Joint3 (deg )")
                # if curr_joints[3] < 0:
                #     if float(curr_joints[3] / pi * 180.0) <= -100:
                #         self.window.label_4.setText("Joint4 (deg )")
                #     else:
                #         self.window.label_4.setText("Joint4 (deg )")
                # else:
                #     if float(curr_joints[3] / pi * 180.0) >= 100:
                #         self.window.label_4.setText("Joint4 ( deg )")
                #     else:
                #         self.window.label_4.setText("Joint4 ( deg )")
                # if curr_joints[4] < 0:
                #     if float(curr_joints[4] / pi * 180.0) <= -100:
                #         self.window.label_5.setText("Joint5 (deg )")
                #     else:
                #         self.window.label_5.setText("Joint5 (deg )")
                # else:
                #     if float(curr_joints[3] / pi * 180.0) >= 100:
                #         self.window.label_5.setText("Joint5 (deg )")
                #     else:
                #         self.window.label_5.setText("Joint5 (deg )")
                # if curr_joints[5] < 0:
                #     if float(curr_joints[5] / pi * 180.0) <= -100:
                #         self.window.label_6.setText("Joint6 (deg )")
                #     else:
                #         self.window.label_6.setText("Joint6 ( deg )")
                # else:
                #     if float(curr_joints[5] / pi * 180.0) >= 100:
                #         self.window.label_6.setText("Joint6 ( deg )")
                #     else:
                #         self.window.label_6.setText("Joint6 (deg )")
                # if curr_joints[6] < 0:
                #     if float(curr_joints[6] / pi * 180.0) <= -100:
                #         self.window.label_7.setText("Joint7 (deg )")
                #     else:
                #         self.window.label_7.setText("Joint7 ( deg )")
                # else:
                #     if float(curr_joints[6] / pi * 180.0) >= 100:
                #         self.window.label_7.setText("Joint7 (  deg )")
                #     else:
                #         self.window.label_7.setText("Joint7 (  deg )")

            # 位置刷新显示
            if movel_m_cm_flag is 0:
                if curr_pose[0] < 0:
                    self.window.label_9.setText("Pose X (%.1f cm )" % float(curr_pose[0] * 100))
                else:
                    self.window.label_9.setText("Pose X ( %.1f cm )" % float(curr_pose[0] * 100))
                if curr_pose[1] < 0:
                    self.window.label_10.setText("Pose Y (%.1f cm )" % float(curr_pose[1] * 100))
                else:
                    self.window.label_10.setText("Pose Y ( %.1f cm )" % float(curr_pose[1] * 100))
                if curr_pose[2] < 0:
                    self.window.label_11.setText("Pose Z (%.1f cm )" % float(curr_pose[2] * 100))
                else:
                    self.window.label_11.setText("Pose Z ( %.1f cm )" % float(curr_pose[2] * 100))
            else:
                if curr_pose[0] < 0:
                    self.window.label_9.setText("Pose X (%.3f m )" % float(curr_pose[0]))
                else:
                    self.window.label_9.setText("Pose X ( %.3f m )" % float(curr_pose[0]))
                if curr_pose[1] < 0:
                    self.window.label_10.setText("Pose Y (%.3f m )" % float(curr_pose[1]))
                else:
                    self.window.label_10.setText("Pose Y ( %.3f m )" % float(curr_pose[1]))
                if curr_pose[2] < 0:
                    self.window.label_11.setText("Pose Z (%.3f m )" % float(curr_pose[2]))
                else:
                    self.window.label_11.setText("Pose Z ( %.3f m )" % float(curr_pose[2]))

            if movel_rad_deg_flag is 0:
                if curr_pose[3] < 0:
                    self.window.label_12.setText("Pose R (%.3f rad )" % float(curr_pose[3]))
                else:
                    self.window.label_12.setText("Pose R ( %.3f rad )" % float(curr_pose[3]))
                if curr_pose[4] < 0:
                    self.window.label_13.setText("Pose P (%.3f rad )" % float(curr_pose[4]))
                else:
                    self.window.label_13.setText("Pose P ( %.3f rad )" % float(curr_pose[4]))
                if curr_pose[5] < 0:
                    self.window.label_14.setText("Pose Y (%.3f rad )" % float(curr_pose[5]))
                else:
                    self.window.label_14.setText("Pose Y ( %.3f rad )" % float(curr_pose[5]))
            else:
                if curr_pose[3] < 0:
                    if float(curr_pose[3] / pi * 180.0) <= -100:
                        self.window.label_12.setText("Pose R (%.2f deg )" % float(curr_pose[3] / pi * 180.0))
                    else:
                        self.window.label_12.setText("Pose R (%.3f deg )" % float(curr_pose[3] / pi * 180.0))
                else:
                    if float(curr_pose[3] / pi * 180.0) >= 100:
                        self.window.label_12.setText("Pose R ( %.2f deg )" % float(curr_pose[3] / pi * 180.0))
                    else:
                        self.window.label_12.setText("Pose R ( %.3f deg )" % float(curr_pose[3] / pi * 180.0))
                if curr_pose[4] < 0:
                    if float(curr_pose[4] / pi * 180.0) <= -100:
                        self.window.label_13.setText("Pose P (%.2f deg )" % float(curr_pose[4] / pi * 180.0))
                    else:
                        self.window.label_13.setText("Pose P (%.3f deg )" % float(curr_pose[4] / pi * 180.0))
                else:
                    if float(curr_pose[4] / pi * 180.0) >= 100:
                        self.window.label_13.setText("Pose P ( %.2f deg )" % float(curr_pose[4] / pi * 180.0))
                    else:
                        self.window.label_13.setText("Pose P ( %.3f deg )" % float(curr_pose[4] / pi * 180.0))
                if curr_pose[5] < 0:
                    if float(curr_pose[5] / pi * 180.0) <= -100:
                        self.window.label_14.setText("Pose Y (%.2f deg )" % float(curr_pose[5] / pi * 180.0))
                    else:
                        self.window.label_14.setText("Pose Y (%.3f deg )" % float(curr_pose[5] / pi * 180.0))
                else:
                    if float(curr_pose[5] / pi * 180.0) >= 100:
                        self.window.label_14.setText("Pose Y ( %.2f deg )" % float(curr_pose[5] / pi * 180.0))
                    else:
                        self.window.label_14.setText("Pose Y ( %.3f deg )" % float(curr_pose[5] / pi * 180.0))

            r.sleep()

    def stop(self):
        self.terminate()


class UpdateThread(QtCore.QThread):
    """
    界面刷新线程，通过信号通知主窗口类中实现的刷新函数
    这很重要，通过将主窗口实例传入界面刷新线程实例的方法不可行
    """
    update_signal = pyqtSignal()

    def __init__(self):
        super(UpdateThread, self).__init__()

    def __del__(self):
        self.wait()

    def run(self):
        r = rospy.Rate(5)  # 2hz
        while not rospy.is_shutdown():
            self.update_signal.emit()
            r.sleep()
    
    def stop(self):
        self.terminate()


class MyWindow(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.center()  # 窗口居中
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)  # 窗口置顶
        self.setWindowIcon(QIcon(os.path.join(curr_path, "robot.ico")))
        self.setupUi(self)
        self.go_to_busy = False
        # 按钮状态标志位
        self.power_flag = False

        # 获取步长
        mode = self.comboBox.currentText()[5:8]
        if mode == 'rad':
            self.joint_step = float(self.comboBox.currentText()[0:4])
        elif mode == 'deg':
            self.joint_step = float(self.comboBox.currentText()[0:4]) * (pi / 180.0)
        print("Init joint_step: %.2frad (%.2fdeg)" % (self.joint_step, self.joint_step * (180.0 / pi)))

        mode = self.comboBox_xyz.currentText()[5:8]
        if mode == 'm':
            self.xyz_step = float(self.comboBox_xyz.currentText()[0:4])
        elif mode == 'cm':
            self.xyz_step = float(self.comboBox_xyz.currentText()[0:4]) * 0.01
        print("Init xyz_step: %.2fm (%.2fcm)" % (self.joint_step, self.joint_step * 0.01))

        mode = self.comboBox_rpy.currentText()[5:8]
        if mode == 'rad':
            self.rpy_step = float(self.comboBox_rpy.currentText()[0:4])
        elif mode == 'deg':
            self.rpy_step = float(self.comboBox_rpy.currentText()[0:4]) * (pi / 180.0)
        print("Init rpy_step: %.2frad (%.2fdeg)" % (self.joint_step, self.joint_step * (180.0 / pi)))

        # 状态保存相关参数
        self.fp = open('joint_states' + '.xml', 'a+')
        self.fp.write('\n<new joint_states/>\n')
        self.group_state_prefix = 'cali_'
        self.group = 'arm'
        self.save_cnt = 0

        # 启动界面刷新进程
        self.update_thread = UpdateThread()  # 创建线程
        self.update_thread.update_signal.connect(self.update)  # 连接信号
        self.update_thread.start()

    def __del__(self):
        self.fp.close()  # 关闭文件

    # 键盘事件
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Enter:
            self.emergency_stop()

        if event.key() == Qt.Key_S:
            self.emergency_stop()

        if event.key() == Qt.Key_Escape:
            sys.exit()  # 退出程序

        if event.key() == Qt.Key_Q:
            sys.exit()  # 退出程序

    # 鼠标事件
    def mousePressEvent(self, event):
        if event.button() == Qt.MidButton:
            self.emergency_stop()

        if event.button() == Qt.RightButton:
            self.emergency_stop()

    # 窗口居中
    def center(self):
        screen = QDesktopWidget().screenGeometry()
        size = self.geometry()
        # self.move((screen.width() - size.width()) / 2,
        #           (screen.height() - size.height()) / 2)
        # self.move((screen.width()), (screen.height() - size.height()))
        self.move((screen.width() - size.width()*2/3), (screen.height() - size.height()*1.3))

    # 使能按钮
    def power(self):
        # 尝试获得锁，写command_arr中的数据
        command_arr_mutex.lock()
        command_arr.data[0] = 1  # 使能指令位置一
        command_arr_mutex.unlock()

        if self.power_flag is False:
            print("[INFO] Power on.")
            self.power_flag = True
            # 设置字体加粗
            font = QtGui.QFont()
            font.setBold(True)
            font.setPointSize(14)
            self.powerButton.setFont(font)
            self.powerButton.setText("Power ON")
        else:
            print("[INFO] Power off.")
            self.power_flag = False
            # 设置字体加粗
            font = QtGui.QFont()
            font.setBold(False)
            font.setPointSize(14)
            self.powerButton.setFont(font)
            self.powerButton.setText("Power OFF")

    @staticmethod
    def emergency_stop():
        print("[INFO] Emergency stop.")

        # 尝试获得锁，写command_arr中的数据
        command_arr_mutex.lock()
        command_arr.data[3] = 1  # 急停指令位置一
        command_arr_mutex.unlock()

    # movej角度显示切换
    def movej_rad_deg(self):
        global movej_rad_deg_flag
        if movej_rad_deg_flag is 0:
            movej_rad_deg_flag = 1
            self.radDegButton_1.setText("deg")
        else:
            movej_rad_deg_flag = 0
            self.radDegButton_1.setText("rad")

    # movel角度显示切换
    def movel_rad_deg(self):
        global movel_rad_deg_flag
        if movel_rad_deg_flag is 0:
            movel_rad_deg_flag = 1
            self.radDegButton_2.setText("deg")
        else:
            movel_rad_deg_flag = 0
            self.radDegButton_2.setText("rad")

    # movel距离显示切换
    def movel_m_cm(self):
        global movel_m_cm_flag
        if movel_m_cm_flag is 0:
            movel_m_cm_flag = 1
            self.mCmButton_1.setText("m")
        else:
            movel_m_cm_flag = 0
            self.mCmButton_1.setText("cm")

    @staticmethod
    def reset_arm():
        print("[INFO] Reset arm done.")
        # 尝试获得锁,写command_arr中的数据
        command_arr_mutex.lock()
        command_arr.data[1] = 1  # 复位指令位置一
        command_arr_mutex.unlock()

    def back_home(self):
        global back_home_flag
        self.backHomeButton.setEnabled(False)  # 禁用此按钮直到运动完成
        self.horizontalSlider.setEnabled(False)  # 禁用速度设置滑块
        print("[INFO] Arm back home request.")
        back_home_flag = True

    def back_home_enable(self):  # 恢复被禁用的按钮
        self.backHomeButton.setEnabled(True)
        self.horizontalSlider.setEnabled(True)

    def set_home(self):
        global goal_joints
        print("[INFO] Arm set home.")
        # 尝试获得锁,写command_arr中的数据
        command_arr_mutex.lock()
        command_arr.data[2] = 1  # 置零指令位置一
        command_arr_mutex.unlock()
        # 保存零位差值
        create_group_state("home_diff" + str(self.save_cnt), self.group, curr_joints, self.fp)
        print("[INFO] home difference: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" %
              (curr_joints[0], curr_joints[1], curr_joints[2], curr_joints[3], curr_joints[4], curr_joints[5], curr_joints[6]))
        print("[INFO] home difference have been saved to joint_states.xml")
        goal_joints = [0.000000] * 7  # 目标位置置零

    # 状态保存按钮
    def save_state(self):
        global curr_joints
        self.save_cnt += 1
        create_group_state(self.group_state_prefix + str(self.save_cnt), self.group, curr_joints, self.fp)
        print("[INFO] joint states: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" %
             (curr_joints[0], curr_joints[1], curr_joints[2], curr_joints[3], curr_joints[4], curr_joints[5], curr_joints[6]))
        print("[INFO] joint states have been saved to joint_states.xml")

    def slide_moved(self):
        global change_vel, vel_scaling
        value = self.horizontalSlider.value()
        change_vel = True  # 速度调整标志置位
        vel_scaling = float(value) / 100.0  # 更新当前速度比例    
        self.label_8.setText("Speed {:d}%".format(value))

    def movej_step_change(self):
        mode = self.comboBox.currentText()[5:8]
        if mode == 'rad':
            self.joint_step = float(self.comboBox.currentText()[0:4])
        elif mode == 'deg':
            self.joint_step = float(self.comboBox.currentText()[0:4]) * (pi / 180.0)
        print("Current joint_step: %.2frad (%.2fdeg)" % (self.joint_step, self.joint_step * (180.0 / pi)))

    def movexyz_step_change(self):
        mode = self.comboBox_xyz.currentText()[5:8]
        if mode == 'm':
            self.xyz_step = float(self.comboBox_xyz.currentText()[0:4])
        elif mode == 'cm':
            self.xyz_step = float(self.comboBox_xyz.currentText()[0:4]) * 0.01
        print("Current xyz_step: %.2fm (%.2fcm)" % (self.xyz_step, self.xyz_step * 100))

    def moverpy_step_change(self):
        mode = self.comboBox_rpy.currentText()[5:8]
        if mode == 'rad':
            self.rpy_step = float(self.comboBox_rpy.currentText()[0:4])
        elif mode == 'deg':
            self.rpy_step = float(self.comboBox_rpy.currentText()[0:4]) * (pi / 180.0)
        print("Current rpy_step: %.2frad (%.2fdeg)" % (self.rpy_step, self.rpy_step * (180.0 / pi)))

    def joint1_minus(self):
        index = 0
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint1_minus_done():
        joint_ctl_arr[0] = 0

    def joint1_plus(self):
        index = 0
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置
            pass

    @staticmethod
    def joint1_plus_done():
        joint_ctl_arr[0] = 0

    def joint2_minus(self):
        index = 1
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint2_minus_done():
        joint_ctl_arr[1] = 0

    def joint2_plus(self):
        index = 1
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint2_plus_done():
        joint_ctl_arr[1] = 0

    def joint3_minus(self):
        index = 2
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint3_minus_done():
        joint_ctl_arr[2] = 0

    def joint3_plus(self):
        index = 2
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint3_plus_done():
        joint_ctl_arr[2] = 0

    def joint4_minus(self):
        index = 3
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint4_minus_done():
        joint_ctl_arr[3] = 0

    def joint4_plus(self):
        index = 3
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint4_plus_done():
        joint_ctl_arr[3] = 0

    def joint5_minus(self):
        index = 4
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint5_minus_done():
        joint_ctl_arr[4] = 0

    def joint5_plus(self):
        index = 4
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint5_plus_done():
        joint_ctl_arr[4] = 0

    def joint6_minus(self):
        index = 5
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint6_minus_done():
        joint_ctl_arr[5] = 0

    def joint6_plus(self):
        index = 5
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint6_plus_done():
        joint_ctl_arr[5] = 0

    def joint7_minus(self):
        index = 6
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] -= self.joint_step  # 操作当前关节目标位置

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint7_minus_done():
        joint_ctl_arr[6] = 0

    def joint7_plus(self):
        index = 6
        global moveJ, goal_joints
        if not moveJ:
            moveJ = True
            goal_joints = list(curr_joints)  # copy curr_joints
            goal_joints[index] += self.joint_step  # 操作当前关节目标位置

    @staticmethod
    def joint7_plus_done():
        joint_ctl_arr[6] = 0

    def x_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 0
            movel_value = -self.xyz_step

    def x_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 0
            movel_value = self.xyz_step

    def y_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 1
            movel_value = -self.xyz_step

    def y_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 1
            movel_value = self.xyz_step

    def z_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 2
            movel_value = -self.xyz_step

    def z_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 2
            movel_value = self.xyz_step

    def roll_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 3
            movel_value = -self.rpy_step

    def roll_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 3
            movel_value = self.rpy_step

    def pitch_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 4
            movel_value = -self.rpy_step

    def pitch_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 4
            movel_value = self.rpy_step

    def yaw_minus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 5
            movel_value = -self.rpy_step

    def yaw_plus(self):
        global moveL, movel_axis, movel_value
        if not moveL:
            moveL = True
            movel_axis = 5
            movel_value = self.rpy_step
    
    # 界面数据刷新
    def update(self):
        # 关节角刷新显示
        if movej_rad_deg_flag is 0:
            if curr_joints[0] < 0:
                self.label_1.setText("Joint1 (%.3f rad )" % float(curr_joints[0]))
            else:
                self.label_1.setText("Joint1 ( %.3f rad )" % float(curr_joints[0]))
            if curr_joints[1] < 0:
                self.label_2.setText("Joint2 (%.3f rad )" % float(curr_joints[1]))
            else:
                self.label_2.setText("Joint2 ( %.3f rad )" % float(curr_joints[1]))
            if curr_joints[2] < 0:
                self.label_3.setText("Joint3 (%.3f rad )" % float(curr_joints[2]))
            else:
                self.label_3.setText("Joint3 ( %.3f rad )" % float(curr_joints[2]))
            if curr_joints[3] < 0:
                self.label_4.setText("Joint4 (%.3f rad )" % float(curr_joints[3]))
            else:
                self.label_4.setText("Joint4 ( %.3f rad )" % float(curr_joints[3]))
            if curr_joints[4] < 0:
                self.label_5.setText("Joint5 (%.3f rad )" % float(curr_joints[4]))
            else:
                self.label_5.setText("Joint5 ( %.3f rad )" % float(curr_joints[4]))
            if curr_joints[5] < 0:
                self.label_6.setText("Joint6 (%.3f rad )" % float(curr_joints[5]))
            else:
                self.label_6.setText("Joint6 ( %.3f rad )" % float(curr_joints[5]))
            if curr_joints[6] < 0:
                self.label_7.setText("Joint7 (%.3f rad )" % float(curr_joints[6]))
            else:
                self.label_7.setText("Joint7 ( %.3f rad )" % float(curr_joints[6]))

        else:
            curr_joints_deg = list(curr_joints)
            for i in range(7):  # 弧度转换为度
                curr_joints_deg[i] = round(float(curr_joints_deg[i] / pi * 180.0), 2)

            if curr_joints_deg[0] < 0:
                if curr_joints_deg[0] <= -100:
                    self.label_1.setText("Joint1 ({:.2f} deg )".format(curr_joints_deg[0]))
                else:
                    self.label_1.setText("Joint1 ({:.3f} deg )".format(curr_joints_deg[0]))
            else:
                if curr_joints_deg[0] >= 100:
                    self.label_1.setText("Joint1 ( {:.2f} deg )".format(curr_joints_deg[0]))
                else:
                    self.label_1.setText("Joint1 ( {:.3f} deg )".format(curr_joints_deg[0]))
            if curr_joints_deg[1] < 0:
                if curr_joints_deg[1] <= -100:
                    self.label_2.setText("Joint2 ({:.2f} deg )".format(curr_joints_deg[1]))
                else:
                    self.label_2.setText("Joint2 ({:.3f} deg )".format(curr_joints_deg[1]))
            else:
                if curr_joints_deg[1] >= 100:
                    self.label_2.setText("Joint2 ( {:.2f} deg )".format(curr_joints_deg[1]))
                else:
                    self.label_2.setText("Joint2 ( {:.3f} deg )".format(curr_joints_deg[1]))
            if curr_joints_deg[2] < 0:
                if curr_joints_deg[2] <= -100:
                    self.label_3.setText("Joint3 ({:.2f} deg )".format(curr_joints_deg[2]))
                else:
                    self.label_3.setText("Joint3 ({:.3f} deg )".format(curr_joints_deg[2]))
            else:
                if curr_joints_deg[2] >= 100:
                    self.label_3.setText("Joint3 ( {:.2f} deg )".format(curr_joints_deg[2]))
                else:
                    self.label_3.setText("Joint3 ( {:.3f} deg )".format(curr_joints_deg[2]))
            if curr_joints_deg[3] < 0:
                if curr_joints_deg[3] <= -100:
                    self.label_4.setText("Joint4 ({:.2f} deg )".format(curr_joints_deg[3]))
                else:
                    self.label_4.setText("Joint4 ({:.3f} deg )".format(curr_joints_deg[3]))
            else:
                if curr_joints_deg[3] >= 100:
                    self.label_4.setText("Joint4 ( {:.2f} deg )".format(curr_joints_deg[3]))
                else:
                    self.label_4.setText("Joint4 ( {:.3f} deg )".format(curr_joints_deg[3]))
            if curr_joints_deg[4] < 0:
                if curr_joints_deg[4] <= -100:
                    self.label_5.setText("Joint5 ({:.2f} deg )".format(curr_joints_deg[4]))
                else:
                    self.label_5.setText("Joint5 ({:.3f} deg )".format(curr_joints_deg[4]))
            else:
                if curr_joints_deg[4] >= 100:
                    self.label_5.setText("Joint5 ( {:.2f} deg )".format(curr_joints_deg[4]))
                else:
                    self.label_5.setText("Joint5 ( {:.3f} deg )".format(curr_joints_deg[4]))
            if curr_joints_deg[5] < 0:
                if curr_joints_deg[5] <= -100:
                    self.label_6.setText("Joint6 ({:.2f} deg )".format(curr_joints_deg[5]))
                else:
                    self.label_6.setText("Joint6 ({:.3f} deg )".format(curr_joints_deg[5]))
            else:
                if curr_joints_deg[5] >= 100:
                    self.label_6.setText("Joint6 ( {:.2f} deg )".format(curr_joints_deg[5]))
                else:
                    self.label_6.setText("Joint6 ( {:.3f} deg )".format(curr_joints_deg[5]))
            if curr_joints_deg[6] < 0:
                if curr_joints_deg[6] <= -100:
                    self.label_7.setText("Joint7 ({:.2f} deg )".format(curr_joints_deg[6]))
                else:
                    self.label_7.setText("Joint7 ({:.3f} deg )".format(curr_joints_deg[6]))
            else:
                if curr_joints_deg[6] >= 100:
                    self.label_7.setText("Joint7 ( {:.2f} deg )".format(curr_joints_deg[6]))
                else:
                    self.label_7.setText("Joint7 ( {:.3f} deg )".format(curr_joints_deg[6]))


        # 位置刷新显示
        if movel_m_cm_flag is 0:
            if curr_pose[0] < 0:
                self.label_9.setText("Pose X (%.1f cm )" % float(curr_pose[0] * 100))
            else:
                self.label_9.setText("Pose X ( %.1f cm )" % float(curr_pose[0] * 100))
            if curr_pose[1] < 0:
                self.label_10.setText("Pose Y (%.1f cm )" % float(curr_pose[1] * 100))
            else:
                self.label_10.setText("Pose Y ( %.1f cm )" % float(curr_pose[1] * 100))
            if curr_pose[2] < 0:
                self.label_11.setText("Pose Z (%.1f cm )" % float(curr_pose[2] * 100))
            else:
                self.label_11.setText("Pose Z ( %.1f cm )" % float(curr_pose[2] * 100))
        else:
            if curr_pose[0] < 0:
                self.label_9.setText("Pose X (%.3f m )" % float(curr_pose[0]))
            else:
                self.label_9.setText("Pose X ( %.3f m )" % float(curr_pose[0]))
            if curr_pose[1] < 0:
                self.label_10.setText("Pose Y (%.3f m )" % float(curr_pose[1]))
            else:
                self.label_10.setText("Pose Y ( %.3f m )" % float(curr_pose[1]))
            if curr_pose[2] < 0:
                self.label_11.setText("Pose Z (%.3f m )" % float(curr_pose[2]))
            else:
                self.label_11.setText("Pose Z ( %.3f m )" % float(curr_pose[2]))

        if movel_rad_deg_flag is 0:
            if curr_pose[3] < 0:
                self.label_12.setText("Pose R (%.3f rad )" % float(curr_pose[3]))
            else:
                self.label_12.setText("Pose R ( %.3f rad )" % float(curr_pose[3]))
            if curr_pose[4] < 0:
                self.label_13.setText("Pose P (%.3f rad )" % float(curr_pose[4]))
            else:
                self.label_13.setText("Pose P ( %.3f rad )" % float(curr_pose[4]))
            if curr_pose[5] < 0:
                self.label_14.setText("Pose Y (%.3f rad )" % float(curr_pose[5]))
            else:
                self.label_14.setText("Pose Y ( %.3f rad )" % float(curr_pose[5]))
        else:
            if curr_pose[3] < 0:
                if float(curr_pose[3] / pi * 180.0) <= -100:
                    self.label_12.setText("Pose R (%.2f deg )" % float(curr_pose[3] / pi * 180.0))
                else:
                    self.label_12.setText("Pose R (%.3f deg )" % float(curr_pose[3] / pi * 180.0))
            else:
                if float(curr_pose[3] / pi * 180.0) >= 100:
                    self.label_12.setText("Pose R ( %.2f deg )" % float(curr_pose[3] / pi * 180.0))
                else:
                    self.label_12.setText("Pose R ( %.3f deg )" % float(curr_pose[3] / pi * 180.0))
            if curr_pose[4] < 0:
                if float(curr_pose[4] / pi * 180.0) <= -100:
                    self.label_13.setText("Pose P (%.2f deg )" % float(curr_pose[4] / pi * 180.0))
                else:
                    self.label_13.setText("Pose P (%.3f deg )" % float(curr_pose[4] / pi * 180.0))
            else:
                if float(curr_pose[4] / pi * 180.0) >= 100:
                    self.label_13.setText("Pose P ( %.2f deg )" % float(curr_pose[4] / pi * 180.0))
                else:
                    self.label_13.setText("Pose P ( %.3f deg )" % float(curr_pose[4] / pi * 180.0))
            if curr_pose[5] < 0:
                if float(curr_pose[5] / pi * 180.0) <= -100:
                    self.label_14.setText("Pose Y (%.2f deg )" % float(curr_pose[5] / pi * 180.0))
                else:
                    self.label_14.setText("Pose Y (%.3f deg )" % float(curr_pose[5] / pi * 180.0))
            else:
                if float(curr_pose[5] / pi * 180.0) >= 100:
                    self.label_14.setText("Pose Y ( %.2f deg )" % float(curr_pose[5] / pi * 180.0))
                else:
                    self.label_14.setText("Pose Y ( %.3f deg )" % float(curr_pose[5] / pi * 180.0))


if __name__ == "__main__":
    rospy.init_node('mantra_hmi_pro')

    app = QApplication(sys.argv)

    ui = Ui_Form()
    window = MyWindow()

    thread_sub = SubThread()
    thread_sub.start()  # 启动消息订阅线程

    thread_pub = PubThread()
    thread_pub.start()  # 启动消息发布线程

    thread_move = MoveThread()
    thread_move.back_home_signal.connect(window.back_home_enable)  # 设置槽函数
    thread_move.start()  # 启动关节运动线程

    # thread_window = WindowThread(window)
    # thread_window.start()  # 启动界面刷新线程

    window.show()  # 界面显示

    thread_pub.exit()
    thread_sub.exit()
    thread_move.exit()
    # thread_window.exit()

    sys.exit(app.exec_())




