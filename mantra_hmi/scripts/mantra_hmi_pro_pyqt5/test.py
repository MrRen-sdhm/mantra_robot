#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  : mantra_gui.py

from __future__ import print_function

from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys


def timerTest():
    class WinForm(QWidget):
        def __init__(self, parent=None):
            super(WinForm, self).__init__(parent)
            self.setWindowTitle("QTimer demo")
            self.listFile = QListWidget()
            self.label = QLabel('显示当前时间')
            self.startBtn = QPushButton('开始')
            self.endBtn = QPushButton('结束')
            layout = QGridLayout(self)

            # 初始化一个定时器
            self.timer = QTimer(self)
            # showTime()方法
            self.timer.timeout.connect(self.showTime)  #

            layout.addWidget(self.label, 0, 0, 1, 2)
            layout.addWidget(self.startBtn, 1, 0)
            layout.addWidget(self.endBtn, 1, 1)

            self.startBtn.clicked.connect(self.startTimer)  # 首先按下start开始计时，到了2s后，触发timeout 信号
            self.endBtn.clicked.connect(self.endTimer)

            self.setLayout(layout)
            QTimer.singleShot(5000, self.showmsg)  # 给定的时间间隔后，调用一个槽函数来发射信号

        def showmsg(self):
            QMessageBox.information(self, '提示信息', '你好')

        def showTime(self):
            # 获取系统现在的时间
            time = QDateTime.currentDateTime()  # 获取当前时间
            # 设置系统时间显示格式
            timeDisplay = time.toString("yyyy-MM-dd hh:mm:ss dddd")
            # 在标签上显示时间
            self.label.setText(timeDisplay)

        def startTimer(self):
            # 设置计时间隔并启动
            self.timer.start(1000)  # 时间为 1 s
            self.startBtn.setEnabled(False)
            self.endBtn.setEnabled(True)

        def endTimer(self):
            self.timer.stop()
            self.startBtn.setEnabled(True)  # 依次循环
            self.endBtn.setEnabled(False)

    if __name__ == "__main__":
        app = QApplication(sys.argv)
        form = WinForm()
        form.show()
        #         QTimer.singleShot(5000, app.quit)    #界面运行5秒后，自动关闭
        sys.exit(app.exec_())


def threadTest2():
    global sec
    sec = 0

    class WorkThread(QThread):  # 自定义一个线程
        trigger = pyqtSignal()

        def __int__(self):
            super(WorkThread, self).__init__()

        def run(self):
            for i in range(2000000000):
                print('haha ' + str(i))  # 此处放运行 量较大的程序

            # 循环完毕后发出信号
            self.trigger.emit()

    def countTime():
        global sec
        sec += 1
        # LED显示数字+1
        lcdNumber.display(sec)

    def work():
        # 计时器每秒计数
        timer.start(1000)  # 每秒运行完后，是lcd增加数字
        # 计时开始
        workThread.start()
        # 当获得循环完毕的信号时，停止计数
        workThread.trigger.connect(timeStop)

    def timeStop():
        timer.stop()
        print("运行结束用时", lcdNumber.value())
        global sec
        sec = 0

    if __name__ == "__main__":
        app = QApplication(sys.argv)
        top = QWidget()
        top.resize(300, 120)

        # 垂直布局类QVBoxLayout
        layout = QVBoxLayout(top)
        # 加个显示屏
        lcdNumber = QLCDNumber()
        layout.addWidget(lcdNumber)
        button = QPushButton("测试")
        layout.addWidget(button)

        timer = QTimer()
        workThread = WorkThread()

        button.clicked.connect(work)
        # 每次计时结束，触发 countTime
        timer.timeout.connect(countTime)

        top.show()
        sys.exit(app.exec_())

# timerTest()

threadTest2()