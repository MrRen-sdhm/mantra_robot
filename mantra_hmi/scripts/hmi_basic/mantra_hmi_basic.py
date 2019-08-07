#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description:
# Date       : 20/05/2019 2:45 PM
# File Name  : mantra_gui.py

from __future__ import print_function

import sys

# ROS相关
import rospy
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

from ui_mantra_hmi import *

# QT相关
from PyQt4.QtGui import QDesktopWidget

# 变量定义
command_arr = Int32MultiArray()
command_cnt = 3
command_arr.data = [0]*command_cnt  # 0:使能 1:复位 2:置零
joint_ctl_arr = [0]*7


class PubThread(QtCore.QThread):
    def __init__(self):
        super(PubThread, self).__init__()
        self.pub = rospy.Publisher('mantra_hmi', Int32MultiArray, queue_size=1)

    def run(self):
        global running, command_cnt
        r = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            try:
                self.pub.publish(command_arr)
            except rospy.exceptions.ROSException:
                print ("Stop publish.")
                break
            # print(command_arr.data)

            # 消息已发送, 置位
            command_arr.data = [0]*command_cnt

            r.sleep()

    def stop(self):
        self.terminate()


class MyWindow(QtGui.QMainWindow, Ui_Form):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.center()  # 窗口居中
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)  # 窗口置顶
        self.setupUi(self)
        self.go_to_busy = False
        # 按钮状态标志位
        self.power_flag = False
        # 微调启用标志位
        self.max_step = 0.1
        self.min_step = 0.05
        self.step = 0.2
        # 状态保存相关参数
        self.fp = open('joint_states' + '.xml', 'a')
        self.fp.write('\n<new joint_states/>\n')
        self.group_state_prefix = 'cali_'
        self.group = 'arm'
        self.save_cnt = 0

    # 窗口居中
    def center(self):
        screen = QDesktopWidget().screenGeometry()
        size = self.geometry()
        # self.move((screen.width() - size.width()) / 2,
        #           (screen.height() - size.height()) / 2)
        self.move((screen.width()), (screen.height() - size.height()))

    # 使能按钮
    def power(self):
        command_arr.data[0] = 1  # 使能指令位置一
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
    def reset_arm():
        print("[INFO] Reset arm done.")
        command_arr.data[1] = 1  # 复位指令位置一

    @staticmethod
    def set_home():
        global goal_positions
        print("[INFO] Arm set home.")
        command_arr.data[2] = 1  # 置零指令位置一
        goal_positions = [0.000000] * 7  # 目标位置置零


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    widget = QtGui.QWidget()
    ui = Ui_Form()
    window = MyWindow()

    rospy.init_node('mantra_hmi_basic')

    thread_pub = PubThread()
    thread_pub.start()  # 启动消息发布线程

    print("You can move the robot after press the power on button now!")

    window.show()  # 界面显示

    thread_pub.exit()
    sys.exit(app.exec_())




