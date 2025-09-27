#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from my_robot_control.msg import DesiredJointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class KinematicSim:
    """
    Kinematic Command Publisher:
    - Nhận lệnh vị trí/vận tốc từ các nguồn khác nhau.
    - Chuyển đổi lệnh thành một thông điệp JointTrajectory.
    - Xuất bản thông điệp này đến arm_controller trong Gazebo.
    """

    def __init__(self):
        rospy.loginfo("Starting kinematic_cmd_publisher (now sending commands to Gazebo)")

        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.dt = 1.0 / float(self.rate_hz)

        # 5 khớp tay chính
        self.arm_joint_names = rospy.get_param(
            "~arm_joint_names",
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        )
        
        # 8 khớp SR
        self.sr_joint_names = rospy.get_param(
            "~sr_joint_names",
            ['joint_sr_11','joint_sr_21','joint_sr_12','joint_sr_22',
             'joint_sr_13','joint_sr_23','joint_sr_14','joint_sr_24']
        )

        # Wheel (chỉ để RViz có TF đầy đủ)
        self.wheel_joint_names = ['joint_banh_phai', 'joint_banh_trai']
        self.all_names = self.arm_joint_names + self.sr_joint_names + self.wheel_joint_names

        # Trạng thái nội bộ
        self.q_arm = np.zeros(len(self.arm_joint_names), dtype=float)
        self.v_arm = np.zeros_like(self.q_arm)
        self.q_arm_cmd = None
        self.q_sr = np.zeros(len(self.sr_joint_names), dtype=float)
        self.q_wheels = np.zeros(len(self.wheel_joint_names), dtype=float)
        self.v_wheels = np.zeros_like(self.q_wheels)
        
        # Kp / vmax để bám setpoint position mượt
        self.kp_pos = rospy.get_param("~kp_pos", 5.0)
        self.max_vel = rospy.get_param("~max_vel", 1.5)

        # Joint limits
        self.joint_limits_min = rospy.get_param("~limits_min", [])
        self.joint_limits_max = rospy.get_param("~limits_max", [])
        if len(self.joint_limits_min) != len(self.arm_joint_names): self.joint_limits_min = None
        if len(self.joint_limits_max) != len(self.arm_joint_names): self.joint_limits_max = None

        # Subs
        self.sub_vel = rospy.Subscriber(
            "/joint_group_vel_controller/command",
            Float64MultiArray, self.cb_arm_vel, queue_size=1)
        self.sub_sr_pos = rospy.Subscriber(
            "/sr_joints_position_controller/command",
            Float64MultiArray, self.cb_sr_pos, queue_size=1)
        self.sub_des = rospy.Subscriber(
            "/desired_joint_state",
            DesiredJointState, self.cb_desired, queue_size=1)
        
        # Pub (thay thế pub /joint_states)
        self.pub_cmd = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        self.pub_js = rospy.Publisher("/joint_states", JointState, queue_size=10)

        rospy.loginfo("KinematicSim ready. rate=%.1f Hz. Publishing /arm_controller/command", self.rate_hz)
        self.loop()

    # ===== callbacks =====
    def cb_arm_vel(self, msg: Float64MultiArray):
        if len(msg.data) >= len(self.v_arm):
            self.v_arm = np.array(msg.data[:len(self.v_arm)], dtype=float)

    def cb_sr_pos(self, msg: Float64MultiArray):
        if len(msg.data) >= len(self.q_sr):
            self.q_sr = np.array(msg.data[:len(self.q_sr)], dtype=float)

    def cb_desired(self, msg: DesiredJointState):
        if len(msg.name) == len(self.arm_joint_names):
            name_to_idx = {n:i for i,n in enumerate(msg.name)}
            try:
                q_cmd = np.array([msg.position[name_to_idx[n]] for n in self.arm_joint_names], dtype=float)
                self.q_arm_cmd = q_cmd
            except KeyError:
                pass
        elif len(msg.position) == len(self.arm_joint_names):
            self.q_arm_cmd = np.array(msg.position, dtype=float)

    # ===== integration + publish =====
    def integrate_arm(self):
        if self.q_arm_cmd is not None:
            err = self.q_arm_cmd - self.q_arm
            v_cmd = np.clip(self.kp_pos * err, -self.max_vel, self.max_vel)
            self.q_arm += v_cmd * self.dt
        else:
            self.q_arm += self.v_arm * self.dt

        if self.joint_limits_min is not None and self.joint_limits_max is not None:
            self.q_arm = np.minimum(self.q_arm, np.array(self.joint_limits_max))
            self.q_arm = np.maximum(self.q_arm, np.array(self.joint_limits_min))

    def publish_trajectory_command(self):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.q_arm)
        point.time_from_start = rospy.Duration(self.dt)
        traj.points.append(point)
        
        self.pub_cmd.publish(traj)

    def publish_dummy_joint_states(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.all_names
        pos = list(self.q_arm) + list(self.q_sr) + list(self.q_wheels)
        vel = list(self.v_arm) + [0.0]*len(self.q_sr) + list(self.v_wheels)
        js.position = pos
        js.velocity = vel
        self.pub_js.publish(js)

    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.integrate_arm()
            self.q_wheels += self.v_wheels * self.dt
            
            # Publish commands to Gazebo
            self.publish_trajectory_command()
            
            # This is optional and only for RViz visualization
            # You can remove this line if Gazebo is running and publishing /joint_states
            self.publish_dummy_joint_states()
            
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kinematic_cmd_publisher")
    try:
        KinematicSim()
    except rospy.ROSInterruptException:
        pass
