#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  : mantra_gui.py

from __future__ import division

import sys
import rospy
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

from ui_mantra_hmi import *
from save_states import create_group_state

from PyQt4.QtGui import QPalette
from PyQt4.QtCore import Qt
from PyQt4.QtGui import QApplication, QWidget, QDesktopWidget
from PyQt4.QtCore import QTimer

array = Int32MultiArray()
array.data = [0]*8  # 0-7:关节角度*100 8:机器人复位信号
joint_ctl_arr = [0]*7


class Thread(QtCore.QThread):
    def __init__(self):
        super(Thread, self).__init__()

    def run(self):
        r = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            pub.publish(array)
            print(array.data)

            # 复位消息已发送, 置位
            if array.data[7] is 1:
                array.data[7] = 0

            r.sleep()

    def stop(self):
        self.terminate()


class MyWindow(QtGui.QMainWindow, Ui_Form):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.center()  # 窗口居中
        self.setupUi(self)
        self.timer = QTimer(self)
        QtCore.QTimer.connect(self.timer, QtCore.SIGNAL("timeout()"), self.timeout)
        self.timer.start(200)  # 0.2s 0.1rad->36/(2*3.14)度=5.73度 5.73/0.2=28.66度每秒=0.5rad每秒
        self.time_out_cnt = 1
        # 微调启用标志位
        self.fine_tuning_flag = False
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
        self.move((screen.width() - size.width()) / 2,
                  (screen.height() - size.height()) / 2)

    # 定时器中断
    def timeout(self):
        if self.time_out_cnt >= 100 / self.horizontalSlider.value():
            self.time_out_cnt = 1
            for i in range(7):
                # 调整角度
                if joint_ctl_arr[i] != 0:
                    if joint_ctl_arr[i] == -1:
                        if self.fine_tuning_flag:
                            array.data[i] -= 1
                        else:
                            array.data[i] -= 10
                    elif joint_ctl_arr[i] == 1:
                        if self.fine_tuning_flag:
                            array.data[i] += 1
                        else:
                            array.data[i] += 10
                    # 限幅
                    if array.data[i] < -314:
                        array.data[i] = -314
                    if array.data[i] > 314:
                        array.data[i] = 314
                    # 修改label
                    if i == 0:
                        if array.data[i] < 0:
                            self.label_1.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_1.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 1:
                        if array.data[i] < 0:
                            self.label_2.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_2.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 2:
                        if array.data[i] < 0:
                            self.label_3.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_3.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 3:
                        if array.data[i] < 0:
                            self.label_4.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_4.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 4:
                        if array.data[i] < 0:
                            self.label_5.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_5.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 5:
                        if array.data[i] < 0:
                            self.label_6.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_6.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))
                    elif i == 6:
                        if array.data[i] < 0:
                            self.label_7.setText("Joint%d (%.2f rad )" % (i+1, float(array.data[i]/100.0)))
                        else:
                            self.label_7.setText("Joint%d ( %.2f rad )" % (i+1, float(array.data[i]/100.0)))

        self.time_out_cnt += 1

    # 微调按钮
    def fine_tuning(self):
        if self.fine_tuning_flag is False:
            print("[INFO] Fine tuning.")
            self.fine_tuning_flag = True
            # self.pushButton_2.setStyleSheet("color: red") # 设置文本颜色
            # 设置字体加粗
            font = QtGui.QFont()
            font.setBold(True)
            self.finetuneButton.setFont(font)
        else:
            print("[INFO] Exit fine tuning.")
            self.fine_tuning_flag = False
            # self.pushButton_2.setStyleSheet("") # 恢复默认文本颜色
            # 设置字体加粗
            font = QtGui.QFont()
            font.setBold(False)
            self.finetuneButton.setFont(font)

    # 状态保存按钮
    def save_state(self):
        self.save_cnt += 1
        joint_states = array.data[0:7]
        create_group_state(self.group_state_prefix + str(self.save_cnt), self.group, joint_states, self.fp)
        print("[INFO] joint states have been saved to joint_states.xml")

    def slide_moved(self):
        value = self.horizontalSlider.value()
        self.label_8.setText("Speed Zoom {:d}%".format(value))

    def reset_robot(self):
        print("[INFO] Reset robot done.")
        array.data = [0, 0, 0, 0, 0, 0, 0, 1]  # 状态置零, 复位置一
        self.label_1.setText("Joint1 ( 0.00 rad )")
        self.label_2.setText("Joint2 ( 0.00 rad )")
        self.label_3.setText("Joint3 ( 0.00 rad )")
        self.label_4.setText("Joint4 ( 0.00 rad )")
        self.label_5.setText("Joint5 ( 0.00 rad )")
        self.label_6.setText("Joint6 ( 0.00 rad )")
        self.label_7.setText("Joint7 ( 0.00 rad )")

    def joint1_minus(self):
        joint_ctl_arr[0] = -1
        if self.fine_tuning_flag:
            array.data[0] -= 1
        else:
            array.data[0] -= 10

        if array.data[0] < -314:
            array.data[0] = -314

        if array.data[0] < 0:
            self.label_1.setText("Joint1 (%.2f rad )" % float(array.data[0]/100.0))
        else:
            self.label_1.setText("Joint1 ( %.2f rad )" % float(array.data[0]/100.0))

    @staticmethod
    def joint1_minus_done():
        joint_ctl_arr[0] = 0

    def joint1_plus(self):
        joint_ctl_arr[0] = 1
        if self.fine_tuning_flag:
            array.data[0] += 1
        else:
            array.data[0] += 10

        if array.data[0] > 314:
            array.data[0] = 314

        if array.data[0] < 0:
            self.label_1.setText("Joint1 (%.2f rad )" % float(array.data[0]/100.0))
        else:
            self.label_1.setText("Joint1 ( %.2f rad )" % float(array.data[0]/100.0))

    @staticmethod
    def joint1_plus_done():
        joint_ctl_arr[0] = 0

    def joint2_minus(self):
        joint_ctl_arr[1] = -1
        if self.fine_tuning_flag:
            array.data[1] -= 1
        else:
            array.data[1] -= 10

        if array.data[1] < -314:
            array.data[1] = -314

        if array.data[1] < 0:
            self.label_2.setText("Joint2 (%.2f rad )" % float(array.data[1]/100.0))
        else:
            self.label_2.setText("Joint2 ( %.2f rad )" % float(array.data[1]/100.0))

    @staticmethod
    def joint2_minus_done():
        joint_ctl_arr[1] = 0

    def joint2_plus(self):
        joint_ctl_arr[1] = 1
        if self.fine_tuning_flag:
            array.data[1] += 1
        else:
            array.data[1] += 10

        if array.data[1] > 314:
            array.data[1] = 314

        if array.data[1] < 0:
            self.label_2.setText("Joint2 (%.2f rad )" % float(array.data[1]/100.0))
        else:
            self.label_2.setText("Joint2 ( %.2f rad )" % float(array.data[1]/100.0))

    @staticmethod
    def joint2_plus_done():
        joint_ctl_arr[1] = 0

    def joint3_minus(self):
        joint_ctl_arr[2] = -1
        if self.fine_tuning_flag:
            array.data[2] -= 1
        else:
            array.data[2] -= 10

        if array.data[2] < -314:
            array.data[2] = -314

        if array.data[2] < 0:
            self.label_3.setText("Joint3 (%.2f rad )" % float(array.data[2]/100.0))
        else:
            self.label_3.setText("Joint3 ( %.2f rad )" % float(array.data[2]/100.0))

    @staticmethod
    def joint3_minus_done():
        joint_ctl_arr[2] = 0

    def joint3_plus(self):
        joint_ctl_arr[2] = 1
        if self.fine_tuning_flag:
            array.data[2] += 1
        else:
            array.data[2] += 10

        if array.data[2] > 314:
            array.data[2] = 314

        if array.data[2] < 0:
            self.label_3.setText("Joint3 (%.2f rad )" % float(array.data[2]/100.0))
        else:
            self.label_3.setText("Joint3 ( %.2f rad )" % float(array.data[2]/100.0))

    @staticmethod
    def joint3_plus_done():
        joint_ctl_arr[2] = 0

    def joint4_minus(self):
        joint_ctl_arr[3] = -1
        if self.fine_tuning_flag:
            array.data[3] -= 1
        else:
            array.data[3] -= 10

        if array.data[3] < -314:
            array.data[3] = -314

        if array.data[3] < 0:
            self.label_4.setText("Joint4 (%.2f rad )" % float(array.data[3]/100.0))
        else:
            self.label_4.setText("Joint4 ( %.2f rad )" % float(array.data[3]/100.0))

    @staticmethod
    def joint4_minus_done():
        joint_ctl_arr[3] = 0

    def joint4_plus(self):
        joint_ctl_arr[3] = 1
        if self.fine_tuning_flag:
            array.data[3] += 1
        else:
            array.data[3] += 10

        if array.data[3] > 314:
            array.data[3] = 314

        if array.data[3] < 0:
            self.label_4.setText("Joint4 (%.2f rad )" % float(array.data[3]/100.0))
        else:
            self.label_4.setText("Joint4 ( %.2f rad )" % float(array.data[3]/100.0))

    @staticmethod
    def joint4_plus_done():
        joint_ctl_arr[3] = 0

    def joint5_minus(self):
        joint_ctl_arr[4] = -1
        if self.fine_tuning_flag:
            array.data[4] -= 1
        else:
            array.data[4] -= 10

        if array.data[4] < -314:
            array.data[4] = -314

        if array.data[4] < 0:
            self.label_5.setText("Joint5 (%.2f rad )" % float(array.data[4]/100.0))
        else:
            self.label_5.setText("Joint5 ( %.2f rad )" % float(array.data[4]/100.0))

    @staticmethod
    def joint5_minus_done():
        joint_ctl_arr[4] = 0

    def joint5_plus(self):
        joint_ctl_arr[4] = 1
        if self.fine_tuning_flag:
            array.data[4] += 1
        else:
            array.data[4] += 10

        if array.data[4] > 314:
            array.data[4] = 314

        if array.data[4] < 0:
            self.label_5.setText("Joint5 (%.2f rad )" % float(array.data[4]/100.0))
        else:
            self.label_5.setText("Joint5 ( %.2f rad )" % float(array.data[4]/100.0))

    @staticmethod
    def joint5_plus_done():
        joint_ctl_arr[4] = 0

    def joint6_minus(self):
        joint_ctl_arr[5] = -1
        if self.fine_tuning_flag:
            array.data[5] -= 1
        else:
            array.data[5] -= 10

        if array.data[5] < -314:
            array.data[5] = -314

        if array.data[5] < 0:
            self.label_6.setText("Joint6 (%.2f rad )" % float(array.data[5]/100.0))
        else:
            self.label_6.setText("Joint6 ( %.2f rad )" % float(array.data[5]/100.0))

    @staticmethod
    def joint6_minus_done():
        joint_ctl_arr[5] = 0

    def joint6_plus(self):
        joint_ctl_arr[5] = 1
        if self.fine_tuning_flag:
            array.data[5] += 1
        else:
            array.data[5] += 10

        if array.data[5] > 314:
            array.data[5] = 314

        if array.data[5] < 0:
            self.label_6.setText("Joint6 (%.2f rad )" % float(array.data[5]/100.0))
        else:
            self.label_6.setText("Joint6 ( %.2f rad )" % float(array.data[5]/100.0))

    @staticmethod
    def joint6_plus_done():
        joint_ctl_arr[5] = 0

    def joint7_minus(self):
        joint_ctl_arr[6] = -1
        if self.fine_tuning_flag:
            array.data[6] -= 1
        else:
            array.data[6] -= 10

        if array.data[6] < -314:
            array.data[6] = -314

        if array.data[6] < 0:
            self.label_7.setText("Joint7 (%.2f rad )" % float(array.data[6]/100.0))
        else:
            self.label_7.setText("Joint7 ( %.2f rad )" % float(array.data[6]/100.0))

    @staticmethod
    def joint7_minus_done():
        joint_ctl_arr[6] = 0

    def joint7_plus(self):
        joint_ctl_arr[6] = 1
        if self.fine_tuning_flag:
            array.data[6] += 1
        else:
            array.data[6] += 10

        if array.data[6] > 314:
            array.data[6] = 314

        if array.data[6] < 0:
            self.label_7.setText("Joint7 (%.2f rad )" % float(array.data[6]/100.0))
        else:
            self.label_7.setText("Joint7 ( %.2f rad )" % float(array.data[6]/100.0))

    @staticmethod
    def joint7_plus_done():
        joint_ctl_arr[6] = 0


if __name__ == "__main__":
    rospy.init_node('mantra_control_pub')
    pub = rospy.Publisher('mantra_hmi', Int32MultiArray, queue_size=1)
    thread_pub = Thread()
    thread_pub.start()  # 启动消息发布线程

    app = QtGui.QApplication(sys.argv)
    widget = QtGui.QWidget()
    ui = Ui_Form()
    window = MyWindow()
    window.show()

    thread_pub.exit()
    sys.exit(app.exec_())




