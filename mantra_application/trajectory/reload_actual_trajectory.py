#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from mpl_toolkits.axes_grid1.inset_locator import TransformedBbox, BboxPatch, BboxConnector


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


# draw a bbox of the region of the inset axes in the parent axes and
# connecting lines between the bbox and the inset axes area
# loc1, loc2 : {1, 2, 3, 4} 
def mark_inset(parent_axes, inset_axes, loc1a=1, loc1b=1, loc2a=2, loc2b=2, **kwargs):
    rect = TransformedBbox(inset_axes.viewLim, parent_axes.transData)

    pp = BboxPatch(rect, fill=False, **kwargs)
    parent_axes.add_patch(pp)

    p1 = BboxConnector(inset_axes.bbox, rect, loc1=loc1a, loc2=loc1b, **kwargs)
    inset_axes.add_patch(p1)
    p1.set_clip_on(False)
    p2 = BboxConnector(inset_axes.bbox, rect, loc1=loc2a, loc2=loc2b, **kwargs)
    inset_axes.add_patch(p2)
    p2.set_clip_on(False)

    return pp, p1, p2


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
        # actual_traj_linear = proc_traj(actual_traj_linear)

        actual_traj_linear_old = np.loadtxt("actual/linear_old.csv", delimiter=',')

        actual_traj_parabolic = np.loadtxt("actual/parabolic.csv", delimiter=',')
        # actual_traj_parabolic = proc_traj(actual_traj_parabolic)

        actual_traj_cubic = np.loadtxt("actual/cubic.csv", delimiter=',')
        # actual_traj_cubic = proc_traj(actual_traj_cubic)

        actual_traj_polynomial5 = np.loadtxt("actual/polynomial5.csv", delimiter=',')
        # actual_traj_polynomial5 = proc_traj(actual_traj_polynomial5)

        # 可视化规划轨迹及实际轨迹
        # plot(t_given, q_given, "Origin trajectory", t_traj=t_traj, traj=traj)



        # 绘图
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        point_size = 2 # 实际轨迹点显示大小
        plt.grid('on')

        ax.plot(t_given, q_given[:, 0], linestyle='', marker='o', markersize=6, markeredgecolor='k', markeredgewidth=1.5, markerfacecolor='r')

        ax.plot(actual_traj_linear[:, 0], actual_traj_linear[:, 1], 'g.', markersize=point_size)
        ax.plot(actual_traj_parabolic[:, 0], actual_traj_parabolic[:, 1], 'c.', markersize=point_size)
        ax.plot(actual_traj_cubic[:, 0], actual_traj_cubic[:, 1], 'b.', markersize=point_size)
        ax.plot(actual_traj_polynomial5[:, 0], actual_traj_polynomial5[:, 1], 'm.', markersize=point_size)

        ax.legend(labels=["points", "linear", "parabolic","cubic", "polynomial5"], ncol=1 ,prop={'size':11.5})
        ax.set_xlabel('time (s)', size=13)
        ax.set_ylabel('position (rad)', size=13)

        #########################################  局部放大显示起始点附近的曲线  #####################################
        x, y_1, y_2, y_3, y_4 = actual_traj_linear[:, 0], actual_traj_linear[:, 1], actual_traj_parabolic[:, 1], actual_traj_cubic[:, 1], actual_traj_polynomial5[:, 1]

        # 嵌入绘制局部放大图的坐标系
        axins1 = inset_axes(ax, width="23%", height="42%",loc='lower left', bbox_to_anchor=(0.22, 0.10, 1, 1), bbox_transform=ax.transAxes)

        # 在子坐标系中绘制原始数据
        axins1.plot(actual_traj_linear[:, 0], actual_traj_linear[:, 1], 'g.', markersize=point_size)
        axins1.plot(actual_traj_parabolic[:, 0], actual_traj_parabolic[:, 1], 'c.', markersize=point_size)
        axins1.plot(actual_traj_cubic[:, 0], actual_traj_cubic[:, 1], 'b.', markersize=point_size)
        axins1.plot(actual_traj_polynomial5[:, 0], actual_traj_polynomial5[:, 1], 'm.', markersize=point_size)

        # 绘制参考线
        axins1.vlines(0.085, -0.1, 0.25, colors="k", linestyles="dashed", lw=0.5)

        # 设置放大区间
        zone_left = 0
        zone_right = 120

        # 坐标轴的扩展比例（根据实际数据调整）
        x_ratio = 0.1 # x轴显示范围的扩展比例
        y_ratio = 0.1 # y轴显示范围的扩展比例

        # X轴的显示范围
        xlim0 = x[zone_left]-(x[zone_right]-x[zone_left])*x_ratio
        xlim1 = x[zone_right]+(x[zone_right]-x[zone_left])*x_ratio

        # Y轴的显示范围
        y = np.hstack((y_1[zone_left:zone_right], y_2[zone_left:zone_right], y_3[zone_left:zone_right], y_4[zone_left:zone_right]))
        ylim0 = np.min(y)-(np.max(y)-np.min(y))*y_ratio
        ylim1 = np.max(y)+(np.max(y)-np.min(y))*y_ratio

        # 调整子坐标系的显示范围
        axins1.set_xlim(xlim0, xlim1)
        axins1.set_ylim(ylim0, ylim1)

        # 隐藏y轴
        axins1.set_yticks([])

        # 建立父坐标系与子坐标系的连接线
        # loc1 loc2: 坐标系的四个角
        # 1 (右上) 2 (左上) 3(左下) 4(右下)
        # mark_inset(ax, axins, loc1=4, loc2=1, fc="none", ec='k', lw=1)
        mark_inset(ax, axins1, loc1a=2, loc1b=1, loc2a=3, loc2b=4, fc="none", ec="k")

        #########################################  局部放大显示转折点附近的曲线  #####################################
        # 嵌入绘制局部放大图的坐标系
        axins2 = inset_axes(ax, width="25%", height="35%",loc='lower left', bbox_to_anchor=(0.51, 0.55, 1, 1), bbox_transform=ax.transAxes)

        # 在子坐标系中绘制原始数据
        axins2.plot(actual_traj_linear[:, 0], actual_traj_linear[:, 1], 'g.', markersize=point_size)
        axins2.plot(actual_traj_parabolic[:, 0], actual_traj_parabolic[:, 1], 'c.', markersize=point_size)
        axins2.plot(actual_traj_cubic[:, 0], actual_traj_cubic[:, 1], 'b.', markersize=point_size)
        axins2.plot(actual_traj_polynomial5[:, 0], actual_traj_polynomial5[:, 1], 'm.', markersize=point_size)

        # 绘制参考线
        axins2.hlines(1.57, 1, 3, colors="k", linestyles="dashed", lw=0.5)
        axins2.hlines(1.60, 1, 3, colors="k", linestyles="dashed", lw=0.5)

        # 设置放大区间
        zone_left = 450
        zone_right = zone_left + 160

        # 坐标轴的扩展比例（根据实际数据调整）
        x_ratio = 0.0 # x轴显示范围的扩展比例
        y_ratio = 0.0 # y轴显示范围的扩展比例

        # X轴的显示范围
        xlim0 = x[zone_left]-(x[zone_right]-x[zone_left])*x_ratio
        xlim1 = x[zone_right]+(x[zone_right]-x[zone_left])*x_ratio

        # Y轴的显示范围
        y = np.hstack((y_1[zone_left:zone_right], y_2[zone_left:zone_right], y_3[zone_left:zone_right], y_4[zone_left:zone_right]))
        ylim0 = np.min(y)-(np.max(y)-np.min(y))*y_ratio
        ylim1 = np.max(y)+(np.max(y)-np.min(y))*y_ratio

        # 调整子坐标系的显示范围
        axins2.set_xlim(xlim0, xlim1)
        axins2.set_ylim(ylim0, ylim1)

        # 隐藏x轴
        axins2.set_xticks([])
        axins2.yaxis.tick_right() # 右侧显示y轴刻度

        # 建立父坐标系与子坐标系的连接线
        # loc1 loc2: 坐标系的四个角
        # 1 (右上) 2 (左上) 3(左下) 4(右下)
        # mark_inset(ax, axins2, loc1=4, loc2=1, fc="none", ec='k', lw=1)
        mark_inset(ax, axins2, loc1a=2, loc1b=1, loc2a=3, loc2b=4, fc="none", ec="k")

        plt.savefig('actual_trajectory.svg', dpi=600, bbox_inches='tight')
        # plt.savefig('actual_trajectory.svg', dpi=600, bbox_inches='tight')

        plt.show()


if __name__ == "__main__":
    Demo()
