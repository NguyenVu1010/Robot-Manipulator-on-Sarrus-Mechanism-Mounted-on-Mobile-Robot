#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

# === BẮT BUỘC SỬ DỤNG BACKEND TKAGG ===
import matplotlib
matplotlib.use('TkAgg')
# ========================================

import tkinter as tk
from tkinter import ttk
import sys, threading, csv, os
from datetime import datetime
from threading import Lock

class FullRobotControllerGUI:
    def __init__(self, master):
        self.master = master
        master.title("Full Robot Controller GUI")

        # =====================================================================
        # === 1. THIẾT LẬP KẾT NỐI ROS                                      ===
        # =====================================================================

        # --- A. Cánh tay (Arm) ---
        self.arm_command_topic = "/joint_group_vel_controller/command"
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.arm_pub = rospy.Publisher(self.arm_command_topic, Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # --- B. Đế di động (Base) ---
        self.base_controller_topic = "/diff_drive_controller/cmd_vel"
        self.cmd_vel_pub = rospy.Publisher(self.base_controller_topic, Twist, queue_size=10)
        
        # --- C. MỤC TIÊU (TARGET POSE) ---
        self.target_pose_topic = "/target_pose_array"
        self.target_pose_pub = rospy.Publisher(self.target_pose_topic, Float64MultiArray, queue_size=1, latch=True)

        # =====================================================================
        # === 2. BIẾN LƯU TRỮ TRẠNG THÁI                                     ===
        # =====================================================================

        # --- A. Cánh tay (Arm) ---
        self.joint_vel_limits = {
            'joint_1': (-0.1, 0.1), 'joint_2': (-0.5, 0.5), 'joint_3': (-0.5, 0.5),
            'joint_4': (-0.5, 0.5), 'joint_5': (-0.5, 0.5),
        }
        self.data_lock = Lock()
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.goal_vel_vars = {name: tk.DoubleVar() for name in self.joint_names}
        self.current_pos_labels_vars = {name: tk.StringVar(value="N/A") for name in self.joint_names}
        self.status_var = tk.StringVar(value="Status: Idle")

        # --- B. Đế di động (Base) ---
        self.twist_command = Twist()
        self.linear_speed_var = tk.DoubleVar(value=0.2)
        self.angular_speed_var = tk.DoubleVar(value=0.5)

        # --- C. MỤC TIÊU (TARGET POSE) ---
        self.goal_pos_vars = {axis: tk.StringVar(value="0.0") for axis in ['x', 'y', 'z']}
        self.goal_alpha_var = tk.StringVar(value="0.0")  # alpha

        # --- D. GHI LOG /joint_states ---
        self.record_secs_var = tk.DoubleVar(value=10.0)     # số giây ghi sau khi gửi
        self.auto_record_var = tk.BooleanVar(value=False)  # chỉ ghi khi bật
        self.collecting = False
        self.t_cmd = rospy.Time(0)
        self.rows = []
        self.name_index = None
        self.log_dir = os.path.join(os.path.expanduser("~"), "joint_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        # =====================================================================
        # === 3. THIẾT LẬP GIAO DIỆN VÀ VÒNG LẶP                           ===
        # =====================================================================
        self.create_widgets()
        self.update_gui()
        self.publish_cmd_vel()
        self.publish_arm_vel()
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # --- Tạo hai cột chính ---
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky="ns", padx=(0, 10))
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="ns")

        # --- KHUNG ĐIỀU KHIỂN CÁNH TAY (Bên trái) ---
        arm_frame = ttk.LabelFrame(left_frame, text="Arm Velocity Control", padding="10")
        arm_frame.pack(fill=tk.X)
        ttk.Label(arm_frame, text="Joint", font=("Helvetica", 10, "bold")).grid(column=0, row=0, padx=5, pady=5)
        for i, name in enumerate(self.joint_names):
            row_num = i + 1; min_val, max_val = self.joint_vel_limits[name]
            ttk.Label(arm_frame, text=f"{name} vel").grid(column=0, row=row_num, sticky=tk.W, padx=5)
            slider = ttk.Scale(arm_frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                               variable=self.goal_vel_vars[name], length=250)
            slider.grid(column=1, row=row_num, sticky=(tk.W, tk.E), padx=5)
            goal_label = ttk.Label(arm_frame, text="0.00"); goal_label.grid(column=2, row=row_num, padx=5)

            def make_goal_update_callback(var, lbl):
                def callback(*args): lbl.config(text=f"{var.get():.3f}")
                return callback
            self.goal_vel_vars[name].trace_add("write",
                make_goal_update_callback(self.goal_vel_vars[name], goal_label))

            current_label = ttk.Label(arm_frame, textvariable=self.current_pos_labels_vars[name], width=8)
            current_label.grid(column=3, row=row_num, padx=5)

        # --- Hàng điều khiển ghi log nhanh cho Arm ---
        arm_log_frame = ttk.Frame(arm_frame)
        arm_log_frame.grid(row=len(self.joint_names)+1, column=0, columnspan=4, pady=(8,0), sticky=tk.W)
        ttk.Label(arm_log_frame, text="Record after (s):").pack(side=tk.LEFT, padx=(0,5))
        ttk.Entry(arm_log_frame, textvariable=self.record_secs_var, width=6).pack(side=tk.LEFT)
        ttk.Button(arm_log_frame, text="Record Now (Arm)",
                   command=lambda: self.start_recording(label="arm")).pack(side=tk.LEFT, padx=8)

        # --- KHUNG ĐIỀU KHIỂN ĐẾ DI ĐỘNG (Bên phải) ---
        base_frame = ttk.LabelFrame(right_frame, text="Base Movement Control (Manual)", padding="10")
        base_frame.pack(fill=tk.X, pady=(0, 10))
        speed_frame = ttk.Frame(base_frame); speed_frame.grid(row=0, column=0, columnspan=3, pady=5)
        ttk.Label(speed_frame, text="Linear Speed (m/s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(speed_frame, textvariable=self.linear_speed_var, width=5).pack(side=tk.LEFT)
        ttk.Label(speed_frame, text="Angular Speed (rad/s):").pack(side=tk.LEFT, padx=15)
        ttk.Entry(speed_frame, textvariable=self.angular_speed_var, width=5).pack(side=tk.LEFT)
        button_frame = ttk.Frame(base_frame); button_frame.grid(row=1, column=0, columnspan=3)
        ttk.Button(button_frame, text="▲\nForward", command=self.move_forward).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(button_frame, text="◄ Turn Left", command=self.turn_left).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(button_frame, text="STOP", command=self.stop_base, style="Stop.TButton").grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(button_frame, text="Turn Right ►", command=self.turn_right).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(button_frame, text="▼\nBackward", command=self.move_backward).grid(row=2, column=1, padx=5, pady=5)
        style = ttk.Style(); style.configure("Stop.TButton", foreground="red", font=('Helvetica', '10', 'bold'))

        # =====================================================================
        # --- KHUNG GỬI LỆNH TARGET POSE ---
        # =====================================================================
        target_frame = ttk.LabelFrame(right_frame, text="Send Target Pose Goal", padding="10")
        target_frame.pack(fill=tk.X)

        # --- Dòng nhập vị trí (Position) ---
        pos_frame = ttk.Frame(target_frame)
        pos_frame.pack(pady=5)
        ttk.Label(pos_frame, text="Position (m):").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(pos_frame, text="X:").grid(row=0, column=1)
        ttk.Entry(pos_frame, textvariable=self.goal_pos_vars['x'], width=7).grid(row=0, column=2, padx=(0, 5))
        ttk.Label(pos_frame, text="Y:").grid(row=0, column=3)
        ttk.Entry(pos_frame, textvariable=self.goal_pos_vars['y'], width=7).grid(row=0, column=4, padx=(0, 5))
        ttk.Label(pos_frame, text="Z:").grid(row=0, column=5)
        ttk.Entry(pos_frame, textvariable=self.goal_pos_vars['z'], width=7).grid(row=0, column=6, padx=(0, 5))

        # --- Dòng nhập góc alpha ---
        alpha_frame = ttk.Frame(target_frame)
        alpha_frame.pack(pady=5)
        ttk.Label(alpha_frame, text="Orientation (rad):").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(alpha_frame, text="Alpha:").grid(row=0, column=1)
        ttk.Entry(alpha_frame, textvariable=self.goal_alpha_var, width=7).grid(row=0, column=2, padx=(0, 5))

        # --- Dòng bật/tắt auto-record + thời gian + nút Send ---
        bottom_frame = ttk.Frame(target_frame)
        bottom_frame.pack(pady=5, fill=tk.X)
        ttk.Checkbutton(bottom_frame, text="Auto record after Send",
                        variable=self.auto_record_var).pack(side=tk.LEFT, padx=(0,10))
        ttk.Label(bottom_frame, text="Record after (s):").pack(side=tk.LEFT, padx=(0,5))
        ttk.Entry(bottom_frame, textvariable=self.record_secs_var, width=6).pack(side=tk.LEFT)
        ttk.Button(bottom_frame, text="Send Goal to /target_pose_array",
                   command=self.send_goal_pose, style="Send.TButton").pack(side=tk.LEFT, padx=10)
        style.configure("Send.TButton", foreground="blue", font=('Helvetica', '10', 'bold'))

        # --- Thanh trạng thái (chung) ---
        status_label = ttk.Label(main_frame, textvariable=self.status_var, font=("Helvetica", 10, "italic"))
        status_label.grid(row=3, column=0, columnspan=2, sticky=tk.W, padx=5, pady=(10,0))

    # =====================================================================
    # --- GỬI GOAL POSE (tôn trọng nút Auto record) ---
    # =====================================================================
    def send_goal_pose(self, record_after=None):
        """Publish [x,y,z,alpha]. Nếu record_after=None -> dùng nút Auto record."""
        try:
            pose_array_msg = Float64MultiArray()
            x = float(self.goal_pos_vars['x'].get())
            y = float(self.goal_pos_vars['y'].get())
            z = float(self.goal_pos_vars['z'].get())
            alpha = float(self.goal_alpha_var.get())
            pose_array_msg.data = [x, y, z, alpha]

            # Quyết định có ghi hay không
            if record_after is None:
                do_record = self.auto_record_var.get()  # theo nút mới
            else:
                do_record = bool(record_after)          # ép theo tham số truyền vào

            # Đặt mốc thời gian TRƯỚC khi publish nếu có ghi
            if do_record:
                self.prepare_recording()

            # Publish
            self.target_pose_pub.publish(pose_array_msg)
            rospy.loginfo(
                f"Published target pose array to {self.target_pose_topic}: "
                f"[x={x:.2f}, y={y:.2f}, z={z:.2f}, alpha={alpha:.2f}]"
            )
            self.status_var.set(
                f"Status: Sent Goal X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Alpha={alpha:.2f}"
                + (" (auto-record ON)" if do_record else "")
            )

            # Bắt đầu thu sau khi gửi
            if do_record:
                self.start_recording(label="goal")

        except ValueError:
            rospy.logerr("Invalid input for pose or alpha. Please enter valid numbers.")
            self.status_var.set("Status: Error! Invalid input.")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred while sending goal: {e}")
            self.status_var.set(f"Status: Error! {e}")

    # =====================================================================
    # --- GHI /joint_states: callback & điều khiển ---
    # =====================================================================
    def prepare_recording(self):
        """Reset buffer & đánh dấu thời điểm gửi."""
        with self.data_lock:
            self.rows = []
        self.name_index = None
        self.t_cmd = rospy.Time.now()

    def start_recording(self, label="arm"):
        """Bật chế độ thu trong N giây và lưu CSV."""
        # Nếu gọi trực tiếp (Record Now), đặt mốc ngay trước khi bật
        self.prepare_recording()
        self.collecting = True
        secs = max(0.1, float(self.record_secs_var.get()))
        threading.Thread(target=self._stop_after_and_save, args=(secs, label), daemon=True).start()
        self.status_var.set(f"Status: Recording {secs:.2f}s after {label}...")

    def _stop_after_and_save(self, secs, label):
        rospy.sleep(secs)
        self.collecting = False
        with self.data_lock:
            rows = list(self.rows)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(self.log_dir, f"joint_states_after_{label}_{ts}.csv")
        try:
            with open(save_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + self.joint_names)
                w.writerows(rows)
            rospy.loginfo("Saved %d rows to %s", len(rows), save_path)
            self.status_var.set(f"Status: Saved {len(rows)} rows → {save_path}")
        except Exception as e:
            rospy.logerr("Save CSV failed: %s", e)
            self.status_var.set(f"Status: Save CSV failed: {e}")

    def joint_state_callback(self, msg):
        # Cập nhật label vị trí hiện tại
        with self.data_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.current_positions[name] = msg.position[i]

        # Nếu đang ghi, chỉ lưu các bản tin có stamp >= t_cmd
        if not self.collecting:
            return

        # map thứ tự khớp theo msg.name lần đầu
        if self.name_index is None:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            idx_list = []
            for n in self.joint_names:
                if n in name_to_idx:
                    idx_list.append(name_to_idx[n])
                else:
                    idx_list.append(None)  # khớp không có trong message
            self.name_index = idx_list

        if msg.header.stamp >= self.t_cmd:
            positions = []
            for idx in self.name_index:
                if idx is None or idx >= len(msg.position):
                    positions.append(float('nan'))
                else:
                    positions.append(msg.position[idx])
            with self.data_lock:
                self.rows.append([msg.header.stamp.to_sec()] + positions)

    # =====================================================================
    # --- CÁC HÀM KHÔNG THAY ĐỔI (điều khiển GUI/Robot) ---
    # =====================================================================
    def update_gui(self):
        if not rospy.is_shutdown():
            with self.data_lock:
                for name, pos in self.current_positions.items():
                    self.current_pos_labels_vars[name].set(f"{pos:.2f}")
            self.master.after(100, self.update_gui)

    def publish_arm_vel(self):
        if not rospy.is_shutdown():
            cmd = Float64MultiArray()
            cmd.data = [self.goal_vel_vars[name].get() for name in self.joint_names]
            self.arm_pub.publish(cmd)
            self.master.after(100, self.publish_arm_vel)

    def move_forward(self): self.twist_command.linear.x = self.linear_speed_var.get(); self.twist_command.angular.z = 0.0
    def move_backward(self): self.twist_command.linear.x = -self.linear_speed_var.get(); self.twist_command.angular.z = 0.0
    def turn_left(self): self.twist_command.linear.x = 0.0; self.twist_command.angular.z = self.angular_speed_var.get()
    def turn_right(self): self.twist_command.linear.x = 0.0; self.twist_command.angular.z = -self.angular_speed_var.get()
    def stop_base(self): self.twist_command.linear.x = 0.0; self.twist_command.angular.z = 0.0
    def publish_cmd_vel(self):
        if not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_command)
            self.master.after(100, self.publish_cmd_vel)

    def on_closing(self):
        rospy.loginfo("GUI is closing...")
        self.stop_base(); self.cmd_vel_pub.publish(self.twist_command); rospy.sleep(0.1)
        self.joint_state_sub.unregister()
        self.master.destroy()
        rospy.signal_shutdown("GUI closed")

if __name__ == '__main__':
    try:
        rospy.init_node('full_robot_controller_gui', anonymous=True)
        root = tk.Tk()
        app = FullRobotControllerGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        print("Program interrupted.")
    except Exception as e:
        rospy.logfatal(f"An error occurred: {e}")
        sys.exit(1)

