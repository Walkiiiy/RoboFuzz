#!/usr/bin/env python3
from __future__ import annotations

import pickle
import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image


class PairPublisher(Node):
    def __init__(self) -> None:
        super().__init__("image_proc_pair_publisher")
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)


def build_messages(payload):
    img = Image()
    img.header.stamp.sec = int(payload["header_sec"])
    img.header.stamp.nanosec = int(payload["header_nanosec"])
    img.header.frame_id = str(payload["frame_id"])
    img.height = int(payload["height"])
    img.width = int(payload["width"])
    img.encoding = str(payload["encoding"])
    img.is_bigendian = int(payload["is_bigendian"])
    img.step = int(payload["step"])
    img.data = list(payload["data"])

    info = CameraInfo()
    info.header.stamp = img.header.stamp
    info.header.frame_id = img.header.frame_id
    info.height = img.height
    info.width = img.width
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    fx = float(max(img.width, 1))
    fy = float(max(img.height, 1))
    cx = float(img.width) / 2.0
    cy = float(img.height) / 2.0
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return img, info


def main(argv=None):
    argv = argv or sys.argv
    if len(argv) != 2:
      print("usage: publish_pair.py <payload.pkl>", file=sys.stderr)
      return 2

    with open(argv[1], "rb") as fp:
        payload = pickle.load(fp)

    rclpy.init()
    node = PairPublisher()
    try:
        img, info = build_messages(payload)
        deadline = time.time() + 0.6
        while time.time() < deadline:
            node.info_pub.publish(info)
            node.image_pub.publish(img)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
