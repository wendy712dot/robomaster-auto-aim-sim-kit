#!/usr/bin/env python3
"""Finite-duration fixed-angle probe; always follows it with invalid/hold commands."""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import VisionSendData


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--yaw', type=float, default=0.0, help='Absolute degrees, left positive')
    parser.add_argument('--pitch', type=float, default=0.0, help='Absolute degrees, down positive')
    parser.add_argument('--seconds', type=float, default=2.0)
    parser.add_argument('--stop', action='store_true')
    args = parser.parse_args()
    import math
    if not all(math.isfinite(x) for x in (args.yaw, args.pitch, args.seconds)) or args.seconds <= 0:
        parser.error('Angles and duration must be finite; duration must be positive')
    rclpy.init()
    node = Node('sim_gimbal_probe')
    pub = node.create_publisher(VisionSendData, '/vision_send_data', qos_profile_sensor_data)

    def publish(stop):
        msg = VisionSendData()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        msg.yaw, msg.pitch = args.yaw, args.pitch
        msg.target_state = 0 if stop else 1
        msg.target_distance = -1.0 if stop else 1.0
        pub.publish(msg)

    print(f'Absolute yaw={args.yaw}, pitch={args.pitch}; stop={args.stop}. F5 must be ON.')
    try:
        end = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < end:
            publish(args.stop)
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                publish(True)
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
