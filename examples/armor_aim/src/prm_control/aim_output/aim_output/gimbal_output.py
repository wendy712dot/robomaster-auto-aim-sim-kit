# -*- coding: utf-8 -*-
"""把第二套视觉链路的 AimResult 转为本仓库统一的 VisionSendData。"""
import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import VisionSendData

from .aim_result import (AimResult, angle_between, blend_direction,
                         anchor_tf_command_to_feedback,
                         gimbal_angles_from_world_direction, limit_rate_step,
                         rotate_vector)

NO_SOLUTION_DISTANCE = -1.0


class GimbalOutput:
    """AimResult -> /vision_send_data。"""

    def __init__(self, node: Node, topic: str = "/vision_send_data",
                 publish_command: bool = False,
                 yaw_sign: float = 1.0,
                 yaw_offset_deg: float = 0.0,
                 pitch_offset_deg: float = 0.0,
                 max_yaw_rate_deg_s: float = 0.0,
                 max_pitch_rate_deg_s: float = 0.0,
                 control_gain: float = 1.0,
                 deadband_deg: float = 0.0,
                 allow_fire: bool = False):
        self.node = node
        self.publish_command = publish_command
        self.yaw_sign = yaw_sign
        self.yaw_offset_deg = yaw_offset_deg
        self.pitch_offset_deg = pitch_offset_deg
        # 视觉链路的处理/传输有延迟，限制指令变化率可减少过冲。0 = 不限制。
        self.max_yaw_rate = float(max_yaw_rate_deg_s)
        self.max_pitch_rate = float(max_pitch_rate_deg_s)
        # 控制增益与死区：延迟主导的闭环里按满增益追目标会过冲（云台冲过目标、目标
        # 离开视野）。gain<1 每步只走一部分，误差几何收敛；进入死区后保持不动，
        # 避免在中心附近来回抖动。
        self.control_gain = max(0.05, min(1.0, float(control_gain)))
        self.deadband_deg = max(0.0, float(deadband_deg))
        self.allow_fire = bool(allow_fire)
        self._last_cmd = None
        self._last_time = None
        self.publisher = node.create_publisher(VisionSendData, topic, qos_profile_sensor_data)
        self.last_command = None  # (yaw, pitch, distance, fire) 便于调试输出
        self.published_count = 0
        self.stopped_count = 0
        self.control_id = 0

    def _limit_rate(self, yaw_deg: float, pitch_deg: float, now_s: float):
        """把指令变化率限制在配置范围内（纯逻辑见 aim_result.limit_rate_step）。"""
        if self.max_yaw_rate <= 0.0 and self.max_pitch_rate <= 0.0:
            return yaw_deg, pitch_deg
        if self._last_cmd is None or self._last_time is None:
            self._last_cmd = (yaw_deg, pitch_deg)
            self._last_time = now_s
            return yaw_deg, pitch_deg

        dt = now_s - self._last_time
        last_yaw, last_pitch = self._last_cmd
        yaw_deg, pitch_deg = limit_rate_step(last_yaw, last_pitch, yaw_deg, pitch_deg,
                                             self.max_yaw_rate, self.max_pitch_rate, dt)
        self._last_cmd = (yaw_deg, pitch_deg)
        self._last_time = now_s
        return yaw_deg, pitch_deg

    def _make_msg(self, header) -> VisionSendData:
        msg = VisionSendData()
        if header is not None:
            msg.header = header
        msg.target_type = 0
        msg.delta_pitch = 0.0
        msg.delta_yaw = 0.0
        msg.vel_x = 0.0
        msg.vel_y = 0.0
        msg.vel_yaw = 0.0
        self.control_id += 1
        msg.control_id = float(self.control_id)
        return msg

    def publish_stop(self, header=None, frame_id: str = "map"):
        """目标丢失/无解：发无效状态，让仿真器停止追踪并保持姿态。"""
        msg = self._make_msg(None)
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = frame_id
        msg.target_state = 0
        msg.target_distance = NO_SOLUTION_DISTANCE
        if self.publish_command:
            self.publisher.publish(msg)
        self.last_command = (None, None, NO_SOLUTION_DISTANCE, False)
        self.stopped_count += 1
        # 目标丢失后重新捕获时，从当前实测姿态重新开始限速（避免从陈旧指令缓慢爬行）
        self._last_cmd = None
        self._last_time = None

    def publish_result(self, result: AimResult, camera_to_world_matrix,
                       feedback_yaw_deg: float, feedback_pitch_deg: float,
                       header=None):
        """
        用 TF 求出目标相对当前相机的修正量，再锚定到最新云台反馈角。
        每帧都基于最新姿态和反馈重算，因此误差不会累积（闭环自校正）。
        返回 (yaw_deg, pitch_deg) 或 None。
        """
        if not result.target_valid:
            self.publish_stop(header)
            return None

        # AimResult 里只保留了角度/距离；这里需要原始方向向量，故由角度还原单位向量
        yaw_rel = math.radians(result.yaw_rel_deg)
        pitch_rel = math.radians(result.pitch_rel_deg)
        d_cam = (math.cos(pitch_rel) * math.sin(yaw_rel),
                 -math.sin(pitch_rel),
                 math.cos(pitch_rel) * math.cos(yaw_rel))

        d_world = rotate_vector(camera_to_world_matrix, d_cam)

        # 当前瞄准方向（相机光学轴 +Z 在世界系下），用于增益插值与死区判断
        d_current = rotate_vector(camera_to_world_matrix, (0.0, 0.0, 1.0))
        err_deg = angle_between(d_current, d_world)
        if self.deadband_deg > 0.0 and err_deg <= self.deadband_deg:
            # 已在中心附近：保持当前指向（继续下发同一设定点，避免抖动）
            d_cmd = d_current
        else:
            d_cmd = blend_direction(d_current, d_world, self.control_gain)

        tf_current_angles = gimbal_angles_from_world_direction(d_current)
        tf_command_angles = gimbal_angles_from_world_direction(
            d_cmd, self.yaw_sign, self.yaw_offset_deg, self.pitch_offset_deg)
        if tf_current_angles is None or tf_command_angles is None:
            self.publish_stop(header)
            return None

        yaw_deg, pitch_deg = anchor_tf_command_to_feedback(
            tf_current_angles[0], tf_current_angles[1],
            tf_command_angles[0], tf_command_angles[1],
            feedback_yaw_deg, feedback_pitch_deg)
        yaw_deg, pitch_deg = self._limit_rate(yaw_deg, pitch_deg, self.node.get_clock().now().nanoseconds / 1e9)
        msg = self._make_msg(header)
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.target_state = 2 if result.fire and self.allow_fire else 1
        msg.yaw = float(yaw_deg)
        # 本例程内部 pitch 向上为正；VisionSendData 的 pitch 向下为正。
        msg.pitch = float(-pitch_deg)
        msg.target_distance = float(result.distance_m)
        if self.publish_command:
            self.publisher.publish(msg)
        self.last_command = (yaw_deg, pitch_deg, result.distance_m, result.fire)
        self.published_count += 1
        # 返回实际下发的（可能被速率限制后的）角度，便于调试输出显示真实指令
        return (yaw_deg, pitch_deg)
