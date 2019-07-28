#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  : mantra_gui.py

from __future__ import print_function
# ROS相关
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState

# Moveit相关
from move_group import *

from ui_mantra_hmi import *
from save_states import create_group_state

# QT相关
from PyQt4.QtGui import QDesktopWidget

# 变量定义
command_arr = Int32MultiArray()
command_cnt = 3
command_arr.data = [0]*command_cnt  # 0:使能 1:复位 2:置零
joint_ctl_arr = [0]*7

vel_scaling = 0.25  # 速度调整比例
curr_positions = [0.000000]*7  # 当前位置
goal_positions = [0.000000]*7  # 目标位置
joint_limits = [1.5708, 1.9, 1.5708, 1.9, 1.5708, 1.9, 3.1415]  # 各关节限位

running = False  # 关节运动标志
back_home = False  # 回零点标志
change_vel = False  # 调整速度标志


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


class SubThread(QtCore.QThread):
    def __init__(self):
        super(SubThread, self).__init__()
        self.sub = rospy.Subscriber("joint_states", JointState, self.callback)

    # 获取关节当前位置
    @staticmethod
    def callback(joint_states):
        global curr_positions
        curr_positions = joint_states.position  # 获取新状态

    def stop(self):
        self.terminate()


class MoveThread(QtCore.QThread):
    def __init__(self):
        super(MoveThread, self).__init__()
        self.move_group = MoveGroup()

    def run(self):
        global running, back_home, change_vel
        r = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():

            if running:  # 关节运动
                self.move_group.go_to_joint_state(goal_positions)
                print('curr_positions:', curr_positions)
                print('goal_positions:', goal_positions)
                print('')
                # print ("[INFO] Go to joint state done.")
                running = False  # 标志复位

            if back_home:  # 回零点
                self.move_group.go_to_pose_named("home")
                print ("[INFO] Back home done.")
                back_home = False  # 标志复位

            if change_vel:  # 调整速度
                self.move_group.set_vel_scaling(vel_scaling)
                change_vel = False  # 标志复位

            r.sleep()

    def stop(self):
        self.terminate()


class WindowThread(QtCore.QThread):
    def __init__(self, window_):
        super(WindowThread, self).__init__()
        self.window = window_

    def run(self):
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if curr_positions[0] < 0:
                self.window.label_1.setText("Joint1 (%.3f rad )" % float(curr_positions[0]))
            else:
                self.window.label_1.setText("Joint1 ( %.3f rad )" % float(curr_positions[0]))
            if curr_positions[1] < 0:
                self.window.label_2.setText("Joint2 (%.3f rad )" % float(curr_positions[1]))
            else:
                self.window.label_2.setText("Joint2 ( %.3f rad )" % float(curr_positions[1]))
            if curr_positions[2] < 0:
                self.window.label_3.setText("Joint3 (%.3f rad )" % float(curr_positions[2]))
            else:
                self.window.label_3.setText("Joint3 ( %.3f rad )" % float(curr_positions[2]))
            if curr_positions[3] < 0:
                self.window.label_4.setText("Joint4 (%.3f rad )" % float(curr_positions[3]))
            else:
                self.window.label_4.setText("Joint4 ( %.3f rad )" % float(curr_positions[3]))
            if curr_positions[4] < 0:
                self.window.label_5.setText("Joint5 (%.3f rad )" % float(curr_positions[4]))
            else:
                self.window.label_5.setText("Joint5 ( %.3f rad )" % float(curr_positions[4]))
            if curr_positions[5] < 0:
                self.window.label_6.setText("Joint6 (%.3f rad )" % float(curr_positions[5]))
            else:
                self.window.label_6.setText("Joint6 ( %.3f rad )" % float(curr_positions[5]))
            if curr_positions[6] < 0:
                self.window.label_7.setText("Joint7 (%.3f rad )" % float(curr_positions[6]))
            else:
                self.window.label_7.setText("Joint7 ( %.3f rad )" % float(curr_positions[6]))

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
        self.move((screen.width() - size.width()), (screen.height() - size.height()) / 2)

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
    def back_home():
        global back_home
        print("[INFO] Arm back home request.")
        back_home = True

    @staticmethod
    def set_home():
        global goal_positions
        print("[INFO] Arm set home.")
        command_arr.data[2] = 1  # 置零指令位置一
        goal_positions = [0.000000] * 7  # 目标位置置零

    # 状态保存按钮
    def save_state(self):
        global curr_positions
        self.save_cnt += 1
        create_group_state(self.group_state_prefix + str(self.save_cnt), self.group, curr_positions, self.fp)
        print("[INFO] joint states have been saved to joint_states.xml")

    def slide_moved(self):
        global change_vel, vel_scaling
        value = self.horizontalSlider.value()
        change_vel = True  # 速度调整标志置位
        vel_scaling = float(value)/100.0  # 更新当前速度比例
        self.label_8.setText("Speed {:d}%".format(value))

    def selectionchange(self):
        mode = self.comboBox.currentText()[5:8]
        if mode == QtCore.QString('rad'):
            self.step = float(self.comboBox.currentText()[0:4])
        elif mode == QtCore.QString('deg'):
            self.step = float(self.comboBox.currentText()[0:4]) * (pi/180.0)
        print ("Current step: %.2frad (%.2fdeg)" % (self.step, self.step * (180.0/pi)))

    def joint1_minus(self):
        index = 0
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint1_minus_done():
        joint_ctl_arr[0] = 0

    def joint1_plus(self):
        index = 0
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint1_plus_done():
        joint_ctl_arr[0] = 0

    def joint2_minus(self):
        index = 1
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint2_minus_done():
        joint_ctl_arr[1] = 0

    def joint2_plus(self):
        index = 1
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint2_plus_done():
        joint_ctl_arr[1] = 0

    def joint3_minus(self):
        index = 2
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint3_minus_done():
        joint_ctl_arr[2] = 0

    def joint3_plus(self):
        index = 2
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint3_plus_done():
        joint_ctl_arr[2] = 0

    def joint4_minus(self):
        index = 3
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint4_minus_done():
        joint_ctl_arr[3] = 0

    def joint4_plus(self):
        index = 3
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint4_plus_done():
        joint_ctl_arr[3] = 0

    def joint5_minus(self):
        index = 4
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint5_minus_done():
        joint_ctl_arr[4] = 0

    def joint5_plus(self):
        index = 4
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint5_plus_done():
        joint_ctl_arr[4] = 0

    def joint6_minus(self):
        index = 5
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint6_minus_done():
        joint_ctl_arr[5] = 0

    def joint6_plus(self):
        index = 5
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint6_plus_done():
        joint_ctl_arr[5] = 0

    def joint7_minus(self):
        index = 6
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] -= self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

        joint_ctl_arr[index] = -1

    @staticmethod
    def joint7_minus_done():
        joint_ctl_arr[6] = 0

    def joint7_plus(self):
        index = 6
        global running, goal_positions
        if not running:
            running = True
            for i in range(curr_positions.__len__()):  # 获取当前位置
                goal_positions[i] = curr_positions[i]

            goal_positions[index] += self.step  # 操作当前关节目标位置

            if goal_positions[index] > joint_limits[index]:
                goal_positions[index] = joint_limits[index]

    @staticmethod
    def joint7_plus_done():
        joint_ctl_arr[6] = 0


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    widget = QtGui.QWidget()
    ui = Ui_Form()
    window = MyWindow()

    rospy.init_node('mantra_control_pub')

    thread_sub = SubThread()
    thread_sub.start()  # 启动消息订阅线程

    thread_pub = PubThread()
    thread_pub.start()  # 启动消息发布线程

    thread_move = MoveThread()
    thread_move.start()  # 启动运动线程

    thread_window = WindowThread(window)
    thread_window.start()  # 启动界面刷新线程

    print("You can move the robot after press the power on button now!")

    window.show()  # 界面显示

    thread_pub.exit()
    thread_sub.exit()
    thread_move.exit()
    thread_window.exit()
    sys.exit(app.exec_())




