#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from my_robot_control.msg import DesiredJointState
from std_msgs.msg import Float64MultiArray

ORDER = ['joint_1','joint_2','joint_3','joint_4','joint_5']

def cb(msg):
    name2vel = dict(zip(msg.name, msg.velocity))
    data = [float(name2vel.get(j, 0.0)) for j in ORDER]   # lấy qdot_d theo đúng thứ tự khớp
    rospy.logdebug("send qdot_d: %s", data)
    out = Float64MultiArray(data=data)
    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("bridge_desired_to_vel_ctrl")
    pub = rospy.Publisher("/arm_velocity_controller/command", Float64MultiArray, queue_size=10)
    rospy.Subscriber("/desired_joint_state", DesiredJointState, cb, queue_size=10)
    rospy.spin()

