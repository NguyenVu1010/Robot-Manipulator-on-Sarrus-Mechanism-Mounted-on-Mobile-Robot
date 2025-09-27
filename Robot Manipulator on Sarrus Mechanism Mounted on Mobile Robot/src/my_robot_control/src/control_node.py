#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from my_robot_control.msg import DesiredJointState
from robot_lib.parameters import load_parameters

class JointVelocityControlNode:
    def __init__(self, params):
        rospy.loginfo("Initializing Joint Velocity Control Node...")
        
        gains = params.get('control_gains', {})
        
        # [THAY ĐỔI] Chỉ cần hệ số Kp cho điều khiển vận tốc
        kp_diagonal_values = gains.get('Kp_diagonal', [5.0]*5) # Giá trị mặc định cho Kp có thể nhỏ hơn
        
        # Kiểm tra để chắc chắn rằng chúng ta có đúng 5 giá trị
        if len(kp_diagonal_values) < 5:
            rospy.logerr("Control gains in params file are incorrect. Need 5 values for Kp.")
            # Sử dụng giá trị mặc định an toàn
            self.Kp = np.diag([5.0] * 5)
        else:
            self.Kp = np.diag(kp_diagonal_values[:5])
        
        rospy.loginfo("Using Kp matrix for velocity control (5x5):\n%s", self.Kp)
        
        self.arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.current_positions = None
        
        # [THAY ĐỔI] Publisher mới cho lệnh vận tốc
        # Tên topic này phải khớp với tên controller trong file controllers.yaml của bạn
        self.joint_velocity_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        self.desired_state_sub = rospy.Subscriber('/desired_joint_state', DesiredJointState, self.control_loop_callback, queue_size=1)
        self.pub_erros = rospy.Publisher('/q_errors', Float64MultiArray, queue_size=1)
        rospy.loginfo("Joint Velocity Control Node is running.")

    def joint_state_callback(self, msg):
        try:
            positions = []
            for name in self.arm_joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_positions = np.array(positions)
        except (ValueError, IndexError):
            # Không cần thiết phải log liên tục nếu message không đầy đủ
            pass

    def control_loop_callback(self, msg):
        if self.current_positions is None:
            rospy.logwarn_throttle(1.0, "Velocity control loop waiting for current joint states.")
            return

        # Trạng thái mong muốn từ planner
        q_d = np.array(msg.position)
        q_dot_d = np.array(msg.velocity) # Đây là thành phần feedforward
        print("Desired positions:", q_d)
        # Trạng thái thực tế
        q = self.current_positions
        # [THAY ĐỔI] Công thức điều khiển vận tốc (P + Feedforward)
        position_error = q_d - q
        self.pub_erros.publish(Float64MultiArray(data=position_error.tolist()))
        # Lệnh vận tốc = Vận tốc theo quỹ đạo + Hiệu chỉnh dựa trên sai số vị trí
        velocity_command = q_dot_d + self.Kp @ position_error
        
        # Publish lệnh vận tốc
        cmd_msg = Float64MultiArray()
        cmd_msg.data = velocity_command
        self.joint_velocity_pub.publish(cmd_msg)

if __name__ == '__main__':
    rospy.init_node('control_node')
    try:
        # Giả sử bạn có file params.yaml, nếu không, cung cấp một dict rỗng
        try:
            param_filepath = rospy.get_param('~param_filepath')
            params = load_parameters(param_filepath)
        except KeyError:
            rospy.logwarn("Parameter '~param_filepath' not set. Using default gains.")
            params = {}

        node = JointVelocityControlNode(params)
        rospy.spin()
        
    except Exception as e:
        rospy.logerr("An error occurred in control_node: %s", e)
