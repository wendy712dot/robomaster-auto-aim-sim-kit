#!/usr/bin/env python3
"""User error topic -> VisionSendData, with stale-result/feedback stop protection."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3Stamped
from rm_interfaces.msg import VisionReceiveData, VisionSendData
from control_math import absolute_command


class AimAdapter(Node):
    def __init__(self):
        super().__init__('sim_aim_adapter')
        self.enabled = self.declare_parameter('enable_control', False).value
        self.timeout = self.declare_parameter('timeout_ms', 200).value / 1000.0
        self.max_step = float(self.declare_parameter('max_step_deg', 2.0).value)
        self.deadband = float(self.declare_parameter('deadband_deg', 0.15).value)
        if self.timeout <= 0:
            raise ValueError('timeout_ms must be positive')
        absolute_command(0, 0, 0, 0, self.max_step, self.deadband)
        self.feedback = None
        self.result = None
        self.result_generation = 0
        self.consumed_generation = -1
        self.command = None
        self.sequence = 0
        self.feedback_time = self.result_time = float('-inf')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(VisionSendData, '/vision_send_data', qos)
        self.create_subscription(VisionReceiveData, '/vision_receive_data', self.on_feedback, qos)
        self.create_subscription(Vector3Stamped, '/aim/error', self.on_result, qos)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Control={self.enabled}; /aim/error: x=yaw error deg, y=pitch error deg, z=distance m')

    def on_feedback(self, msg):
        if math.isfinite(msg.yaw) and math.isfinite(msg.pitch):
            self.feedback = (msg.yaw, msg.pitch)
            self.feedback_time = time.monotonic()
        else:
            self.feedback = None

    def on_result(self, msg):
        age = (self.get_clock().now().nanoseconds -
               (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        values = (msg.vector.x, msg.vector.y, msg.vector.z)
        valid = (msg.header.frame_id == 'camera_optical_frame' and
                 0 <= age <= self.timeout and all(math.isfinite(x) for x in values) and
                 msg.vector.z > 0)
        self.result = values if valid else None
        self.result_generation += 1
        # Retain acquisition age: repeatedly publishing the same stamp cannot refresh it.
        self.result_time = time.monotonic() - max(0, age) if valid else float('-inf')

    def tick(self):
        if not self.enabled:
            return
        now = time.monotonic()
        valid = (self.result is not None and self.feedback is not None and
                 now - self.result_time <= self.timeout and
                 now - self.feedback_time <= self.timeout)
        if valid and (self.consumed_generation != self.result_generation or self.command is None):
            self.command = absolute_command(*self.feedback, *self.result[:2],
                                            self.max_step, self.deadband)
            self.consumed_generation = self.result_generation
        elif not valid:
            self.command = None
        msg = VisionSendData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        self.sequence += 1
        msg.control_id = float(self.sequence)
        actionable = valid and self.command is not None
        msg.target_state = 1 if actionable else 0
        msg.target_distance = float(self.result[2]) if actionable else -1.0
        if actionable:
            msg.yaw, msg.pitch = self.command
        elif self.feedback is not None:
            msg.yaw, msg.pitch = self.feedback
        self.publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = AimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.enabled and rclpy.ok():
            node.result = None
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
