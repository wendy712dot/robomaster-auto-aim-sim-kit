#!/usr/bin/env python3
"""Receive raw images + camera calibration. Optional OpenCV display; no detector."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class ImageProbe(Node):
    def __init__(self):
        super().__init__('sim_image_probe')
        self.show = self.declare_parameter('show', False).value
        self.count = 0
        self.start = time.monotonic()
        self.last_image = self.start
        self.warned = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/image_raw', self.image, qos)
        self.create_subscription(CameraInfo, '/camera_info', self.info, qos)
        self.create_timer(1.0, self.report)
        self.calibration = None

    def info(self, msg):
        data = (msg.width, msg.height, tuple(msg.k), tuple(msg.d))
        if data != self.calibration:
            self.calibration = data
            self.get_logger().info(f'CameraInfo {msg.width}x{msg.height}, K={list(msg.k)}, D={list(msg.d)}')

    def image(self, msg):
        self.count += 1
        self.last_image = time.monotonic()
        self.warned = False
        self.latest = f'{msg.width}x{msg.height} {msg.encoding} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        if self.show:
            import cv2
            from cv_bridge import CvBridge
            cv2.imshow('Simulator image', CvBridge().imgmsg_to_cv2(msg, 'bgr8'))
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                rclpy.shutdown()

    def report(self):
        now = time.monotonic()
        if now - self.last_image > 1.0 and not self.warned:
            self.get_logger().warning('No image for >1 s; check simulator, ROS_DOMAIN_ID and QoS.')
            self.warned = True
        if self.count:
            self.get_logger().info(f'{self.latest} receive_fps={self.count / (now - self.start):.1f}')
        self.count = 0
        self.start = now


if __name__ == '__main__':
    rclpy.init()
    node = ImageProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
