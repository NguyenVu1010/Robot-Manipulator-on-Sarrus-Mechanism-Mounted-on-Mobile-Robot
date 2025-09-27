#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from .kinematic_ml import RobotArm  # yêu cầu có forward1(q)->[x,y,z,alpha], jacobian1(q)->(4x5)

def wrap_to_pi(a):
    """Đưa góc về [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

class TrajectoryGenerator:
    """
    Sinh quỹ đạo khớp (q, q_dot) bằng resolved-rate:
      qdot = J^+ xdot_cmd + (I - J^+J) qdot0,   q_{k+1} = q_k + qdot*dt
    Quỹ đạo Cartesian A->B theo hồ sơ vận tốc hình thang; có feedback task-space nhẹ.
    """

    def __init__(self, time_step=0.01, nullspace_gain=0.5, dls_lambda=1e-3, vel_limits=None):
        self.robot_arm = RobotArm()
        self.dt = float(time_step)
        self.k = float(nullspace_gain)     # gain cho qdot0 (null-space bias về giữa giới hạn)
        self.lmbd = float(dls_lambda)      # hệ số DLS ổn định gần kỳ dị

        # Giới hạn khớp
        jl = self.robot_arm.joint_limits  # dict {'joint_1': (min,max), ...}
        mins = [jl[name][0] for name in jl]
        maxs = [jl[name][1] for name in jl]
        self.joint_limits_internal = {
            'min': np.array(mins, dtype=float),
            'max': np.array(maxs, dtype=float),
        }

        # Giới hạn vận tốc (rad/s hoặc m/s với khớp tịnh tiến) — chỉnh cho phù hợp robot của bạn
        if vel_limits is None:
            self.vel_limits = np.array([1.2, 1.2, 1.2, 1.2, 1.2], dtype=float)
        else:
            self.vel_limits = np.asarray(vel_limits, dtype=float)

        # Feedback task-space nhẹ để dán quỹ đạo (đơn vị tương ứng x,y,z[m], alpha[rad])
        self.Kx = np.diag([1.0, 1.0, 1.0, 0.5])

    def _qdot_from_xdot(self, q, xdot):
        """
        Damped Least Squares + null-space bias:
          qdot = J^T (J J^T + λ^2 I)^(-1) xdot + (I - J^+ J) qdot0
        """
        q = np.asarray(q, dtype=float)
        xdot = np.asarray(xdot, dtype=float)

        # Null-space bias kéo q về giữa giới hạn
        q_mid = 0.5 * (self.joint_limits_internal['max'] + self.joint_limits_internal['min'])
        q_rng = (self.joint_limits_internal['max'] - self.joint_limits_internal['min'])
        q_rng_sq = np.maximum(q_rng**2, 1e-6)
        qdot0 = self.k * (q - q_mid) / q_rng_sq

        # Jacobian và DLS pseudoinverse
        J = np.asarray(self.robot_arm.jacobian1(q), dtype=float)   # (4 x 5)
        JJt = J @ J.T
        I_x = np.eye(JJt.shape[0])
        J_pinv = J.T @ np.linalg.solve(JJt + (self.lmbd**2) * I_x, I_x)

        I_q = np.eye(len(q))
        qdot = J_pinv @ xdot + -(I_q - J_pinv @ J) @ qdot0

        # Kẹp vận tốc theo giới hạn
        qdot = np.clip(qdot, -self.vel_limits, self.vel_limits)
        return qdot

    def generate_trajectory(self, start_joints, end_pos, duration):
        """
        Sinh danh sách các cặp (q, q_dot) tại mỗi bước dt để end-effector đi từ
        forward1(start_joints) tới end_pos = [x, y, z, alpha] theo profile hình thang.

        Parameters
        ----------
        start_joints : array-like (len=5)
        end_pos      : array-like (len=4)  [x, y, z, alpha]
        duration     : float > 0

        Returns
        -------
        list[tuple[np.ndarray, np.ndarray]]  # [(q, q_dot), ...], gồm cả mẫu cuối với q_dot=0
        """
        q = np.asarray(start_joints, dtype=float).copy()
        xB = np.asarray(end_pos, dtype=float)
        if duration <= 0:
            raise ValueError("duration phải > 0")

        # Pose đầu và vector đường đi (alpha đi đường ngắn nhất)
        xA = np.asarray(self.robot_arm.forward1(q), dtype=float)
        path_vec = xB - xA
        path_vec[3] = wrap_to_pi(path_vec[3])

        # Profile hình thang (20% - 60% - 20%), tự điều chỉnh nếu quá ngắn
        accel_time = 0.2 * duration
        decel_time = 0.2 * duration
        const_time = duration - accel_time - decel_time
        if const_time < 0:
            rospy.logwarn("duration quá ngắn; tự điều chỉnh profile hình thang.")
            accel_time = decel_time = 0.4 * duration
            const_time = 0.2 * duration

        # v_max chuẩn hóa sao cho ∫ v dt = 1 (để s chạy từ 0→1)
        s_total = (accel_time / 2.0) + const_time + (decel_time / 2.0)
        vmax_n = 1.0 / s_total

        # Bảo đảm có mẫu cuối t = duration
        n_steps = int(round(duration / self.dt))
        timeline = np.linspace(0.0, duration, n_steps + 1)

        traj = []

        for t in timeline:
            # 1) s(t), s_dot(t)
            if t < accel_time:
                a = vmax_n / accel_time
                s     = 0.5 * a * t**2
                s_dot = a * t
            elif t < accel_time + const_time:
                s_acc = 0.5 * vmax_n * accel_time
                s     = s_acc + vmax_n * (t - accel_time)
                s_dot = vmax_n
            else:
                t_dec = t - (accel_time + const_time)
                a = vmax_n / decel_time
                s_acc_const = 0.5 * vmax_n * accel_time + vmax_n * const_time
                s     = s_acc_const + vmax_n * t_dec - 0.5 * a * t_dec**2
                s_dot = vmax_n - a * t_dec

            # 2) Trajectory Cartesian + feedforward vận tốc
            x_d = xA + s * path_vec
            xdot_ff = s_dot * path_vec

            # 3) Feedback task-space nhẹ để bám quỹ đạo (đặc biệt gần kỳ dị/phi tuyến)
            x = np.asarray(self.robot_arm.forward1(q), dtype=float)
            ex = x_d - x
            ex[3] = wrap_to_pi(ex[3])
            xdot_cmd = xdot_ff + self.Kx @ ex

            # 4) qdot từ xdot_cmd (DLS + null-space bias)
            qdot = self._qdot_from_xdot(q, xdot_cmd)

            # 5) Mẫu hiện tại (trước khi cập nhật) & mẫu cuối đứng yên
            if t >= duration - 1e-12:
                qdot = np.zeros_like(qdot)
            traj.append((q.copy(), qdot.copy()))

            # 6) Tích phân q & kẹp giới hạn
            q = q + qdot * self.dt
            q = np.clip(q, self.joint_limits_internal['min'], self.joint_limits_internal['max'])

        return traj

