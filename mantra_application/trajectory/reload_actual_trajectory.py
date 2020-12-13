#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot(t_given, q_given, title, t_traj=None, traj=None):
    fig = plt.figure(figsize=(8, 8))
    plt.subplots_adjust(wspace=0.0, hspace=0.05) # 设置子图间距
    ax1 = plt.subplot(3,1,1)
    plt.plot(t_given, q_given[:, 0], 'ro')
    # 显示插补轨迹
    if t_traj is not None and traj is not None: plt.plot(t_traj, traj[:,0], 'k')

    plt.grid('on')
    #plt.title(title)
    #plt.xlabel('time (s)')
    plt.ylabel('position (rad)', size=13)
    plt.xlim(t_given[0]-0.5, t_given[-1]+0.5)
    # plt.ylim(min(q_given[:, 0]) - 0.1, max(q_given[:, 0]) + 0.1)
    plt.gca().axes.xaxis.set_major_locator(ticker.MultipleLocator(1.0)) # 设置刻度密度
    plt.gca().axes.xaxis.set_ticklabels([])

    ax2 = plt.subplot(3,1,2)
    plt.plot(t_given, q_given[:, 1], 'rh')
    #plt.plot(t_given, q_given[:, 1], 'k')
    # 显示插补轨迹
    if t_traj is not None and traj is not None: plt.plot(t_traj, traj[:,1], 'k')

    plt.grid('on')
    #plt.xlabel('time (s)')
    plt.ylabel('velocity (rad / s)', size=13)
    plt.xlim(t_given[0]-0.5, t_given[-1]+0.5)
    plt.gca().axes.xaxis.set_major_locator(ticker.MultipleLocator(1.0)) # 设置刻度密度
    plt.gca().axes.xaxis.set_ticklabels([])

    ax3 = plt.subplot(3,1,3)
    plt.plot(t_given, q_given[:, 2], 'rd')
    #plt.plot(t_given, q_given[:, 2], 'k')
    # 显示插补轨迹
    if t_traj is not None and traj is not None:
        if title != "Linear interpolation": plt.plot(t_traj, traj[:,2], 'k')  # 线性插补加速度未无穷大，不绘制

    plt.grid('on')
    plt.xlabel('time (s)', size=13)
    plt.ylabel('acceleration (rad / s$^{2}$)', size=13)
    plt.gca().axes.xaxis.set_major_locator(ticker.MultipleLocator(1.0)) # 设置刻度密度
    plt.xlim(t_given[0]-0.5, t_given[-1]+0.5)

    fig.align_ylabels([ax1, ax2, ax3]) # 对齐标签
    plt.savefig('%s.svg' % title, dpi=600, bbox_inches='tight')
    # plt.savefig('%s.pdf' % title, dpi=600, bbox_inches='tight')

    ax1.set_title(title) # 不保存title

    plt.show()


class Demo:
    def __init__(self):
        data = np.loadtxt("trajectory3.csv", skiprows=1, delimiter=',')
        #print data.shape
        time_ls, pos, vel, acc = data[:,0], data[:,1], data[:,2], data[:,3]

        t_given = data[:,0]
        q_given = data[:,[1,2,3]]

        print time_ls

        # 显示原始轨迹
        print "\npos", pos
        print "\nvel", vel
        print "\nacc", acc

        def proc_traj(traj):
            # 寻找与规划的轨迹匹配的实际轨迹
            last_idx = 0
            for i in range(len(traj)):
                if traj[i][0] > t_given[-1]:
                    last_idx = i - 1
                    break
                if i == len(traj) - 1:
                    last_idx = len(traj) - 1

            traj = traj[:last_idx + 1]

            return traj

        # 载入实际轨迹
        actual_traj_linear = np.loadtxt("actual/linear.csv", delimiter=',')
        print actual_traj_linear[-1][0], time_ls[-1]
        # actual_traj_linear = proc_traj(actual_traj_linear)

        actual_traj_parabolic = np.loadtxt("actual/parabolic.csv", delimiter=',')
        # actual_traj_parabolic = proc_traj(actual_traj_parabolic)

        actual_traj_cubic = np.loadtxt("actual/cubic.csv", delimiter=',')
        # actual_traj_cubic = proc_traj(actual_traj_cubic)

        actual_traj_polynomial5 = np.loadtxt("actual/polynomial5.csv", delimiter=',')
        # actual_traj_polynomial5 = proc_traj(actual_traj_polynomial5)

        # 可视化规划轨迹及实际轨迹
        # plot(t_given, q_given, "Origin trajectory", t_traj=t_traj, traj=traj)

        fig = plt.figure() # figsize=(8, 8)
        plt.grid('on')

        plt.plot(t_given, q_given[:, 0], 'ro')
        plt.plot(actual_traj_linear[:, 0], actual_traj_linear[:, 1], 'g.', markersize=4)
        plt.plot(actual_traj_parabolic[:, 0], actual_traj_parabolic[:, 1], 'm.', markersize=4)
        plt.plot(actual_traj_cubic[:, 0], actual_traj_cubic[:, 1], 'b.', markersize=4)
        plt.plot(actual_traj_polynomial5[:, 0], actual_traj_polynomial5[:, 1], 'y.', markersize=4)

        # plt.scatter(actual_traj_polynomial5[:, 0], actual_traj_polynomial5[:, 1], s=1, color='b')

        plt.show()

        # exit()


        # 插值测试
        # Interpolation(time_ls, pos, vel, acc)



if __name__ == "__main__":
    Demo()
