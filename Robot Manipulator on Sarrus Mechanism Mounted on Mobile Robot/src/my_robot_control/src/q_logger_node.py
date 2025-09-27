#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import numpy as np
from typing import Dict, List
from sensor_msgs.msg import JointState
from my_robot_control.msg import DesiredJointState


def fmt(v: float) -> str:
    return f"{v:.6f}"


class QLoggerNode:
    """
    Logger q_d và qdot_d từ /desired_joint_state.
    Tùy chọn: log q thực tế từ /joint_states và sai số.
    Nếu ~gazebo_style=true -> in ra đúng format: 'Desired positions: [ ... ]'
    """

    def __init__(self):
        self.arm_joint_names: List[str] = rospy.get_param(
            "~arm_joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"],
        )
        self.sample_stride: int = int(rospy.get_param("~sample_stride", 5))
        self.log_actual: bool = bool(rospy.get_param("~log_actual", True))
        self.csv_path: str = rospy.get_param("~csv_path", "")
        self.gazebo_style: bool = bool(rospy.get_param("~gazebo_style", False))
        self.log_velocity_line: bool = bool(rospy.get_param("~log_velocity_line", False))

        self._counter = 0
        self._latest_actual: Dict[str, float] = {}

        # CSV
        self._csv = None
        self._csv_writer = None
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self._csv = open(self.csv_path, "w", newline="")
            headers = ["t"] + \
                      [f"q_d[{n}]" for n in self.arm_joint_names] + \
                      [f"qd_d[{n}]" for n in self.arm_joint_names]
            if self.log_actual:
                headers += [f"q[{n}]" for n in self.arm_joint_names]
                headers += [f"e[{n}]=q_d-q" for n in self.arm_joint_names]
            self._csv_writer = csv.writer(self._csv)
            self._csv_writer.writerow(headers)
            rospy.loginfo("q_logger_node: logging CSV to %s", self.csv_path)

        # Subs
        rospy.Subscriber("/desired_joint_state", DesiredJointState,
                         self._cb_desired, queue_size=50)
        if self.log_actual:
            rospy.Subscriber("/joint_states", JointState,
                             self._cb_joint_states, queue_size=50)

        style = "gazebo-style" if self.gazebo_style else "detailed"
        rospy.loginfo("q_logger_node started (stride=%d, style=%s, log_actual=%s)",
                      self.sample_stride, style, self.log_actual)

    def _cb_joint_states(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for n in self.arm_joint_names:
            idx = name_to_idx.get(n)
            if idx is not None and idx < len(msg.position):
                self._latest_actual[n] = float(msg.position[idx])

    def _cb_desired(self, msg: DesiredJointState):
        self._counter += 1
        if self._counter % self.sample_stride != 0:
            return

        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q_d = []
        qd_d = []
        for n in self.arm_joint_names:
            idx = name_to_idx.get(n)
            q_d.append(float(msg.position[idx]) if idx is not None else float("nan"))
            qd_d.append(float(msg.velocity[idx]) if idx is not None else float("nan"))

        t = (msg.header.stamp.to_sec()
             if msg.header.stamp and msg.header.stamp != rospy.Time(0)
             else rospy.Time.now().to_sec())

        if self.gazebo_style:
            # In đúng phong cách gazebo (như print("Desired positions:", q_d) với q_d là numpy array)
            print("Desired positions:", np.array(q_d))
            if self.log_velocity_line:
                print("Desired velocities:", np.array(qd_d))
        else:
            # In chi tiết + sai số (nếu bật)
            line = [f"t={t:.3f}",
                    "q_d=[" + ", ".join(fmt(v) for v in q_d) + "]",
                    "qdot_d=[" + ", ".join(fmt(v) for v in qd_d) + "]"]
            if self.log_actual:
                q_actual = [self._latest_actual.get(n, float("nan")) for n in self.arm_joint_names]
                e = [qd - qa for qd, qa in zip(q_d, q_actual)]
                line += [
                    "q=[" + ", ".join(fmt(v) for v in q_actual) + "]",
                    "e=q_d-q=[" + ", ".join(fmt(v) for v in e) + "]"
                ]
            rospy.loginfo(" | ".join(line))

        if self._csv_writer:
            row = [f"{t:.6f}"] + [f"{v:.6f}" for v in q_d] + [f"{v:.6f}" for v in qd_d]
            if self.log_actual:
                q_actual = [self._latest_actual.get(n, float("nan")) for n in self.arm_joint_names]
                e = [qd - qa for qd, qa in zip(q_d, q_actual)]
                row += [f"{v:.6f}" for v in q_actual] + [f"{v:.6f}" for v in e]
            self._csv_writer.writerow(row)

    def spin(self):
        rospy.spin()

    def __del__(self):
        try:
            if self._csv:
                self._csv.flush()
                self._csv.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("q_logger_node")
    node = QLoggerNode()
    node.spin()

