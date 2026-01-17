#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class AmclFeeder(Node):
    def __init__(self):
        super().__init__("nav2_amcl_feeder")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.timer = self.create_timer(0.2, self.publish_scan)
        self._init_static_tf()

    def _init_static_tf(self):
        broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()

        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = now
        tf_odom_base.header.frame_id = "odom"
        tf_odom_base.child_frame_id = "base_link"
        tf_odom_base.transform.rotation.w = 1.0

        tf_base_scan = TransformStamped()
        tf_base_scan.header.stamp = now
        tf_base_scan.header.frame_id = "base_link"
        tf_base_scan.child_frame_id = "base_scan"
        tf_base_scan.transform.translation.x = 0.2
        tf_base_scan.transform.rotation.w = 1.0

        broadcaster.sendTransform([tf_odom_base, tf_base_scan])

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_scan"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.time_increment = 0.0
        msg.scan_time = 0.2
        msg.range_min = 0.05
        msg.range_max = 10.0
        msg.ranges = [5.0] * 360
        msg.intensities = [0.0] * 360
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AmclFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
