#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, EmptyResponse

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class EETraceTF:
    def __init__(self):
        rospy.init_node("ee_trace_tf")

        # === Params ===
        self.ee_link      = rospy.get_param("~ee_link", "gripper_link")  # đổi cho đúng với URDF
        self.output_frame = rospy.get_param("~output_frame", "base_link") # 'world' / 'odom' nếu có TF
        self.rate_hz      = float(rospy.get_param("~rate", 50.0))
        self.min_dist2    = float(rospy.get_param("~min_dist", 0.003))**2
        self.line_width   = float(rospy.get_param("~line_width", 0.005))
        self.max_points   = int(rospy.get_param("~max_points", 8000))

        # === Pubs ===
        self.pub_pose  = rospy.Publisher("/ee_pose",  PoseStamped, queue_size=10)
        self.pub_trace = rospy.Publisher("/ee_trace", Marker,      queue_size=1, latch=True)

        # === TF2 ===
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # === Marker (LINE_STRIP) trong output_frame ===
        self.points, self.last_pt = [], None
        self.marker = Marker()
        self.marker.header.frame_id = self.output_frame
        self.marker.ns = "ee_trace_tf"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = self.line_width
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a = (0.0, 0.8, 0.2, 1.0)
        self.marker.lifetime = rospy.Duration(0)
        self.marker.pose.orientation.w = 1.0

        rospy.Service("~reset_ee_trace", Empty, self.reset_srv)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self.tick)
        rospy.loginfo("ee_trace_tf: ee_link=%s, output_frame=%s", self.ee_link, self.output_frame)

    def reset_srv(self, _):
        self.points.clear()
        self.last_pt = None
        self.marker.points = []
        self.marker.header.stamp = rospy.Time.now()
        self.pub_trace.publish(self.marker)
        return EmptyResponse()

    def _append_point_if_far(self, x, y, z, stamp):
        from math import sqrt
        if self.last_pt is None:
            ok = True
        else:
            dx = x - self.last_pt[0]; dy = y - self.last_pt[1]; dz = z - self.last_pt[2]
            ok = (dx*dx + dy*dy + dz*dz) >= self.min_dist2
        if ok:
            self.points.append(Point(x=x, y=y, z=z))
            if len(self.points) > self.max_points:
                self.points = self.points[-self.max_points:]
            self.marker.points = self.points
            self.marker.header.stamp = stamp
            self.pub_trace.publish(self.marker)
            self.last_pt = (x, y, z)

    def tick(self, _):
        try:
            # Lấy TF: output_frame <- ee_link
            tf = self.tf_buf.lookup_transform(self.output_frame, self.ee_link,
                                              rospy.Time(0), rospy.Duration(0.2))
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Chưa có TF %s <- %s (%s)",
                                   self.output_frame, self.ee_link, str(e))
            return

        # Xuất /ee_pose (PoseStamped)
        ps = PoseStamped()
        ps.header = tf.header
        ps.header.frame_id = self.output_frame
        # do_transform_pose dùng identity pose ở ee_link
        ee_pose_in_ee = PoseStamped()
        ee_pose_in_ee.header.frame_id = self.ee_link
        ee_pose_in_ee.header.stamp = tf.header.stamp
        ps = do_transform_pose(ee_pose_in_ee, tf)  # pose của gốc ee_link trong output_frame

        self.pub_pose.publish(ps)

        # Thêm điểm vào LINE_STRIP
        p = ps.pose.position
        self._append_point_if_far(p.x, p.y, p.z, ps.header.stamp)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        EETraceTF().spin()
    except rospy.ROSInterruptException:
        pass

