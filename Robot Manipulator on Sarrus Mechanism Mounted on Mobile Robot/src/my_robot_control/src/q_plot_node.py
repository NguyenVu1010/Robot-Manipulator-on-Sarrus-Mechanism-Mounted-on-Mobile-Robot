#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import collections
from sensor_msgs.msg import JointState

# dùng Qt5Agg cho cửa sổ realtime
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class QPlotNode:
    """
    Vẽ q (joint positions) theo thời gian cho 5 khớp tay chính.
    - Sub: /joint_states
    - Params:
        ~arm_joint_names (list[str])  mặc định: ['joint_1'..'joint_5']
        ~window           (float, s)  cửa sổ thời gian, mặc định 20s
        ~plot_rate        (float, Hz) chỉ để ấn định kích thước bộ đệm, mặc định 50
    """

    def __init__(self):
        rospy.loginfo("Starting q_plot_node (realtime plot of joint positions)")

        self.arm_joint_names = rospy.get_param(
            "~arm_joint_names",
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        )
        self.window = float(rospy.get_param("~window", 20.0))
        plot_rate = float(rospy.get_param("~plot_rate", 50.0))
        self.maxlen = int(max(5.0, self.window) * plot_rate)

        self.t = collections.deque(maxlen=self.maxlen)
        self.q = [collections.deque(maxlen=self.maxlen) for _ in self.arm_joint_names]
        self.t0 = rospy.Time.now()

        # Figure
        self.fig, self.ax = plt.subplots()
        self.lines = []
        for name in self.arm_joint_names:
            (ln,) = self.ax.plot([], [], label=name)
            self.lines.append(ln)

        self.ax.set_title("q vs. time")
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("q (rad or m)")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()

        self.sub = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states, queue_size=50)
        self.anim = FuncAnimation(self.fig, self._update_plot, interval=100, blit=False)

    def cb_joint_states(self, msg: JointState):
        # build map name->index
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        # time
        t_now = (msg.header.stamp - self.t0).to_sec() if msg.header.stamp != rospy.Time(0) \
            else (rospy.Time.now() - self.t0).to_sec()
        self.t.append(t_now)

        # append positions for each tracked joint (giữ chiều dài các deque đồng bộ)
        for i, n in enumerate(self.arm_joint_names):
            idx = name_to_idx.get(n, None)
            if idx is not None and idx < len(msg.position):
                self.q[i].append(float(msg.position[idx]))
            else:
                # nếu khớp chưa xuất hiện trong /joint_states, lặp lại giá trị cuối
                last = self.q[i][-1] if len(self.q[i]) else 0.0
                self.q[i].append(last)

    def _update_plot(self, _):
        if not self.t:
            return self.lines

        t = np.array(self.t, dtype=float)
        t_min = max(0.0, t[-1] - self.window)
        self.ax.set_xlim(t_min, t_min + self.window)

        # autoscale y theo toàn bộ dữ liệu đang hiển thị
        y_lo, y_hi = +1e9, -1e9
        for i, ln in enumerate(self.lines):
            qi = np.array(self.q[i], dtype=float)
            # cắt t cho đúng độ dài qi
            ln.set_data(t[-len(qi):], qi)
            if qi.size:
                y_lo = min(y_lo, float(qi.min()))
                y_hi = max(y_hi, float(qi.max()))
        if y_lo < y_hi:
            pad = 0.05 * (y_hi - y_lo + 1e-6)
            self.ax.set_ylim(y_lo - pad, y_hi + pad)

        return self.lines

    def spin(self):
        # chặn ở đây để hiện cửa sổ matplotlib
        plt.show()


if __name__ == "__main__":
    rospy.init_node("q_plot_node")
    try:
        QPlotNode().spin()
    except rospy.ROSInterruptException:
        pass

