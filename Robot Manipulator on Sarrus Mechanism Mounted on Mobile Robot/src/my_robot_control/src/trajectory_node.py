#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from my_robot_control.msg import DesiredJointState

# Resolved-rate generator của bạn
from robot_lib.tinh_qdot import TrajectoryGenerator

def clamp(v, lo, hi): return max(lo, min(v, hi))

class TrajectoryPlannerNode:
    def __init__(self):
        rospy.loginfo("Initializing Trajectory Planner for Gazebo (sending commands to /arm_controller/command)...")

        # State
        self.mode = "IDLE"
        self.final_target_pose_bl = None
        self.current_arm_joints = None
        self.planned_trajectory = []
        self.publish_timer = None

        # Params
        self.arm_trajectory_duration = rospy.get_param('~arm_trajectory_duration', 10.0)
        self.publish_frequency = rospy.get_param('~publish_frequency', 50.0)
        self.arm_joint_names = rospy.get_param('~arm_joint_names', ['joint_1','joint_2','joint_3','joint_4','joint_5'])

        # Trajectory generator (resolved-rate)
        self.dt = 1.0 / self.publish_frequency
        self.trajectory_generator = TrajectoryGenerator(time_step=self.dt)
        rospy.loginfo("TrajectoryGenerator dt=%.4f", self.dt)

        # Pub/Sub
        self.trajectory_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.target_sub = rospy.Subscriber('/target_pose_array', Float64MultiArray, self.target_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)

        # Main timer: 10 Hz -> chuyển trạng thái
        self.main_timer = rospy.Timer(rospy.Duration(0.1), self.main_loop)
        rospy.loginfo("Trajectory Planner ready (Gazebo mode).")

    # ===== Callbacks =====
    def joint_state_callback(self, msg):
        try:
            positions = [msg.position[msg.name.index(j)] for j in self.arm_joint_names]
            self.current_arm_joints = np.array(positions, dtype=float)
        except (ValueError, IndexError):
            pass

    def target_callback(self, msg):
        rospy.loginfo("New target received. Preempting current task...")
        self.cancel_current_task()
        if len(msg.data) != 4:
            rospy.logerr("Target must be [x,y,z,alpha] in base_link, got %d", len(msg.data))
            return
        self.final_target_pose_bl = np.array(msg.data, dtype=float)
        rospy.loginfo("Target (base_link): %s", np.round(self.final_target_pose_bl, 3))
        self.mode = "PLANNING"

    # ===== State machine =====
    def main_loop(self, _):
        if self.mode == "IDLE": return
        if self.final_target_pose_bl is None or self.current_arm_joints is None: return

        if self.mode == "PLANNING":
            self.plan_and_start()
        elif self.mode == "EXECUTING":
            pass

    def plan_and_start(self):
        q0 = self.current_arm_joints.copy()
        target = self.final_target_pose_bl.copy()

        rospy.loginfo("Planning resolved-rate trajectory to pose (base_link): %s", np.round(target,3))
        try:
            rospy.loginfo("Starting trajectory calculation from current joints: %s", np.round(q0, 3))
            
            self.planned_trajectory = self.trajectory_generator.generate_trajectory(
                start_joints=q0,
                end_pos=target,
                duration=self.arm_trajectory_duration
            )

        except ValueError as e:
            rospy.logerr("Trajectory generation failed: %s", e)
            self.cancel_current_task()
            return

        rospy.loginfo("Planning done: %d points. Executing...", len(self.planned_trajectory))
        self.mode = "EXECUTING"
        dt = 1.0 / self.publish_frequency
        self.publish_timer = rospy.Timer(rospy.Duration(dt), self.publish_next_point)

    def publish_next_point(self, _):
        if not self.planned_trajectory:
            rospy.loginfo("Execution finished.")
            self.cancel_current_task()
            return

        q_d, qdot_d = self.planned_trajectory.pop(0)

        # Thêm dòng này để in ra desired point
        rospy.loginfo("Publishing point: q_d=%s, qdot_d=%s", np.round(q_d, 3), np.round(qdot_d, 3))

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(q_d)
        point.velocities = list(qdot_d)
        point.time_from_start = rospy.Duration(self.dt)
        
        traj_msg.points.append(point)

        self.trajectory_pub.publish(traj_msg)

    def cancel_current_task(self):
        if self.publish_timer is not None:
            self.publish_timer.shutdown()
            self.publish_timer = None
        self.planned_trajectory = []
        self.final_target_pose_bl = None
        self.mode = "IDLE"

if __name__ == '__main__':
    rospy.init_node('trajectory_node')
    TrajectoryPlannerNode()
    rospy.spin()
