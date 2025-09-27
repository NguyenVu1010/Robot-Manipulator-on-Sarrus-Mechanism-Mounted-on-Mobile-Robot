# -*- coding: utf-8 -*-
    def update_plot(self, frame):
        """H� m cập nhật chỉ dữ liệu của các đường line (an to� n chiều d� i & 1-điểm)."""
        with self.lock:
            # Tự động pause nếu quá hạn target
            if self.plotting_active and (rospy.Time.now() - self.last_target_time).to_sec() > self.target_timeout:
                rospy.logwarn_throttle(5, f"No target pose received for {self.target_timeout}s. Pausing plot animation.")
                self.plotting_active = False

            if not self.plotting_active:
                return self.all_artists

            # === 1) Đường đi & điểm hiện tại của robot ===
            if self.robot_x_data_full_path and self.robot_y_data_full_path:
                self.robot_path_line.set_data(self.robot_x_data_full_path, self.robot_y_data_full_path)
                # Điểm đơn phải truyền dạng list
                self.robot_current_point.set_data(
                    [self.robot_x_data_full_path[-1]],
                    [self.robot_y_data_full_path[-1]]
                )
            else:
                self.robot_path_line.set_data([], [])
                self.robot_current_point.set_data([], [])

            # === 2) Target (dùng [x, y] từ [x, y, z, alpha]) ===
            if self.target_pose:
                self.target_point.set_data([self.target_pose[0]], [self.target_pose[1]])
            else:
                self.target_point.set_data([], [])

            # === 3) Cửa sổ thời gian & dữ liệu khớp ===
            if self.time_data:
                current_time_end = self.time_data[-1]

                # Tìm start index cho cửa sổ "time_window"
                start_time_idx = 0
                for i, t in enumerate(self.time_data):
                    if t >= current_time_end - self.time_window:
                        start_time_idx = i
                        break

                time_slice = list(self.time_data)[start_time_idx:]

                # Ước lượng dt để nới trục hoặc nhân đôi điểm 1-mẫu
                if len(time_slice) >= 2:
                    # dt trung bình
                    dt_est = max(1e-3, (time_slice[-1] - time_slice[0]) / (len(time_slice) - 1))
                else:
                    dt_est = 0.01  # fallback

                # Đặt trục x an to� n (né identical left==right)
                if time_slice:
                    left = float(time_slice[0])
                    right = float(time_slice[-1])
                    if left == right:
                        right = left + dt_est
                    self.ax2.set_xlim(left, right)
                    self.ax3.set_xlim(left, right)
                else:
                    self.ax2.set_xlim(0, self.time_window)
                    self.ax3.set_xlim(0, self.time_window)

                # Cập nhật từng khớp, đảm bảo len(x) == len(y)
                for name in self.joints_to_plot:
                    pos_slice = list(self.position_data[name])[start_time_idx:]
                    vel_slice = list(self.velocity_data[name])[start_time_idx:]

                    # ---- Positions ----
                    npos = min(len(time_slice), len(pos_slice))
                    if npos == 0:
                        self.joint_pos_lines[name].set_data([], [])
                    elif npos == 1:
                        t0 = time_slice[-1]
                        y0 = pos_slice[-1]
                        self.joint_pos_lines[name].set_data([t0, t0 + dt_est], [y0, y0])
                    else:
                        self.joint_pos_lines[name].set_data(time_slice[-npos:], pos_slice[-npos:])

                    # ---- Velocities ----
                    nvel = min(len(time_slice), len(vel_slice))
                    if nvel == 0:
                        self.joint_vel_lines[name].set_data([], [])
                    elif nvel == 1:
                        t0 = time_slice[-1]
                        y0 = vel_slice[-1]
                        self.joint_vel_lines[name].set_data([t0, t0 + dt_est], [y0, y0])
                    else:
                        self.joint_vel_lines[name].set_data(time_slice[-nvel:], vel_slice[-nvel:])
            else:
                # Chưa có thời gian → xóa dữ liệu khớp
                self.ax2.set_xlim(0, self.time_window)
                self.ax3.set_xlim(0, self.time_window)
                for name in self.joints_to_plot:
                    self.joint_pos_lines[name].set_data([], [])
                    self.joint_vel_lines[name].set_data([], [])

        return self.all_artists

