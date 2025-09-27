#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
send_test_goal.py

Một script đơn giản để publish một mục tiêu (PoseStamped) đến topic /target_pose.
Hữu ích để kiểm tra node điều phối mà không cần gõ lệnh rostopic dài dòng.

Cách sử dụng:
1. Chạy hệ thống robot chính: roslaunch my_robot_control coordinated_control.launch
2. Mở terminal mới, source workspace và chạy script này.

Ví dụ:
# Gửi mục tiêu mặc định (x=0.6, y=0.2, z=0.4)
rosrun my_robot_control send_test_goal.py

# Gửi một mục tiêu tùy chỉnh
rosrun my_robot_control send_test_goal.py --x 0.5 --y -0.1 --z 0.35

# Xem trợ giúp
rosrun my_robot_control send_test_goal.py --help
"""

import rospy
from geometry_msgs.msg import PoseStamped
import argparse
import sys

def send_goal(x, y, z, frame_id="odom"):
    """
    Hàm chính để tạo và publish mục tiêu.
    """
    # Khởi tạo publisher
    # Topic name phải khớp với topic mà trajectory_node đang subscribe
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    
    # Khởi tạo node. anonymous=True đảm bảo node có tên duy nhất.
    rospy.init_node('test_goal_sender', anonymous=True)

    # Chờ một chút để publisher kết nối
    rospy.sleep(1.0)

    # Tạo message PoseStamped
    goal_pose = PoseStamped()
    
    # Điền vào header
    # rospy.Time.now() sẽ tự động sử dụng sim_time nếu /use_sim_time là true
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = frame_id

    # Điền vào vị trí
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    # Điền vào hướng (mặc định là không xoay)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    try:
        rospy.loginfo(f"Publishing goal to topic '/target_pose' in frame '{frame_id}':")
        rospy.loginfo(f"Position (x, y, z): ({x:.2f}, {y:.2f}, {z:.2f})")
        
        pub.publish(goal_pose)
        
        rospy.loginfo("Goal published successfully. Script will exit.")
    
    except rospy.ROSInterruptException:
        rospy.logerr("Publishing failed due to ROS shutdown.")
        pass

if __name__ == '__main__':
    # --- Xử lý tham số dòng lệnh ---
    parser = argparse.ArgumentParser(description="Send a test goal to the coordinated controller.")
    parser.add_argument('--x', type=float, default=-4.0, help="Target X coordinate.")
    parser.add_argument('--y', type=float, default=0.2, help="Target Y coordinate.")
    parser.add_argument('--z', type=float, default=0.6, help="Target Z coordinate.")
    parser.add_argument('--frame', type=str, default="odom", help="The reference frame for the coordinates (e.g., 'odom', 'base_link').")

    # rospy.myargv sẽ loại bỏ các tham số remapping của ROS
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    # Gọi hàm chính với các tham số đã được xử lý
    send_goal(args.x, args.y, args.z, args.frame)