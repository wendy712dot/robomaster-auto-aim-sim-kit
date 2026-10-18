# -*- coding: utf-8 -*-
"""
aim_result.py — 统一瞄准结果（输入/输出抽象的中间表示）

任务书 §五 要求：算法产生统一的瞄准结果（如 AimResult），包含 yaw、pitch、distance
和 target_valid，并明确单位与含义；调试输出与 ROS2 输出都只消费这一份结果，检测器与
PnP 模块不负责拼装 ROS2 消息。

单位与参考系约定（本文件是唯一权威定义）：
  yaw_rel_deg   目标相对【相机光轴】的方位角，单位度；目标在光轴右侧为正
  pitch_rel_deg 目标相对【水平面】的仰角，单位度；向上为正
  distance_m    目标距离，单位米
  target_valid  PnP 解算有效（上位节点给出非零解）

纯计算，无 ROS 依赖，便于单元测试。
"""
from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass
class AimResult:
    yaw_rel_deg: float = 0.0
    pitch_rel_deg: float = 0.0
    distance_m: float = 0.0
    target_valid: bool = False
    fire: bool = False
    status: str = "NO_ARMOR"

    def __str__(self) -> str:
        return (f"AimResult(valid={self.target_valid} status={self.status} "
                f"yaw_rel={self.yaw_rel_deg:+.2f}deg pitch_rel={self.pitch_rel_deg:+.2f}deg "
                f"dist={self.distance_m:.2f}m fire={self.fire})")


def is_zero_solution(x_mm: float, y_mm: float, z_mm: float) -> bool:
    """pose_estimator 用全 0 表示"无装甲/停止"（NO_ARMOR / STOPPING），见 PoseEstimatorNode.cpp。"""
    return x_mm == 0.0 and y_mm == 0.0 and z_mm == 0.0


def aim_result_from_armor_xyz(x_mm: float, y_mm: float, z_mm: float,
                              fire: bool = False,
                              status: str = "NO_ARMOR") -> AimResult:
    """
    由 PnP 输出（相机/枪口坐标系，单位 mm，x 右 / y 下 / z 前）构造 AimResult。

    这是"视线法"(line-of-sight)解算：不做弹道/重力补偿（任务书本轮不要求弹道模型，
    真机的查表修正是实机经验值，不适用于仿真）。
    """
    if is_zero_solution(x_mm, y_mm, z_mm):
        return AimResult(0.0, 0.0, 0.0, False, fire, status)

    distance_mm = math.sqrt(x_mm * x_mm + y_mm * y_mm + z_mm * z_mm)
    if distance_mm <= 0.0:
        return AimResult(0.0, 0.0, 0.0, False, fire, status)

    yaw_rel_deg = math.degrees(math.atan2(x_mm, z_mm))          # 右为正
    horizontal = math.hypot(x_mm, z_mm)
    pitch_rel_deg = math.degrees(math.atan2(-y_mm, horizontal))  # 上为正（图像 y 向下）

    return AimResult(yaw_rel_deg, pitch_rel_deg, distance_mm / 1000.0, True, fire, status)


def unit_vector(x_mm: float, y_mm: float, z_mm: float) -> Optional[Sequence[float]]:
    """相机坐标系下的单位方向向量（指向目标）。"""
    n = math.sqrt(x_mm * x_mm + y_mm * y_mm + z_mm * z_mm)
    if n <= 0.0:
        return None
    return (x_mm / n, y_mm / n, z_mm / n)


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float):
    """四元数 -> 3x3 旋转矩阵（行主序，ROS xyzw 约定）。"""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def rotate_vector(matrix, vec):
    return tuple(sum(matrix[i][j] * vec[j] for j in range(3)) for i in range(3))


def shortest_angle_delta(target_deg: float, current_deg: float) -> float:
    """从 current 转到 target 的最短有向角度差（度），范围 (-180, 180]。"""
    d = target_deg - current_deg
    return (d + 180.0) % 360.0 - 180.0


def anchor_tf_command_to_feedback(tf_current_yaw_deg: float,
                                  tf_current_elevation_deg: float,
                                  tf_command_yaw_deg: float,
                                  tf_command_elevation_deg: float,
                                  feedback_yaw_deg: float,
                                  feedback_pitch_deg: float):
    """把 TF 坐标中的相对修正量锚定到 VisionReceiveData 的角度基准。

    当前仿真器的 TF 光学坐标与 VisionSendData 的绝对角存在固定轴偏置，不能把
    TF 绝对角直接写入消息。yaw 修正取最短圆周差；TF elevation 向上为正，而
    VisionReceiveData.pitch 向下为正。

    返回 (vision_yaw_deg, target_elevation_deg)。输出适配器最后再把 elevation
    转成下正的 VisionSendData.pitch。
    """
    yaw_delta = shortest_angle_delta(tf_command_yaw_deg, tf_current_yaw_deg)
    elevation_delta = tf_command_elevation_deg - tf_current_elevation_deg
    vision_yaw = (feedback_yaw_deg + yaw_delta + 180.0) % 360.0 - 180.0
    target_elevation = -feedback_pitch_deg + elevation_delta
    return vision_yaw, target_elevation


def limit_rate_step(last_yaw_deg, last_pitch_deg, yaw_deg, pitch_deg,
                    max_yaw_rate_deg_s: float, max_pitch_rate_deg_s: float, dt: float):
    """
    指令变化率限制：把 (yaw, pitch) 相对上一指令的增量限制在 max_rate*dt 内。

    用途：视觉链路存在处理/传输延迟，直接下发大幅绝对角容易过冲；限制变化率后
    云台平滑逼近目标。

    返回 (yaw, pitch)。速率 <= 0 表示该轴不限制。yaw 按最短方向处理绕圈。
    """
    dt = max(1e-3, dt)
    out_yaw, out_pitch = yaw_deg, pitch_deg

    if max_yaw_rate_deg_s > 0.0:
        dy = shortest_angle_delta(yaw_deg, last_yaw_deg)
        lim = max_yaw_rate_deg_s * dt
        out_yaw = last_yaw_deg + max(-lim, min(lim, dy))

    if max_pitch_rate_deg_s > 0.0:
        lim = max_pitch_rate_deg_s * dt
        dp = pitch_deg - last_pitch_deg
        out_pitch = last_pitch_deg + max(-lim, min(lim, dp))

    return out_yaw, out_pitch


def angle_between(a, b) -> float:
    """两个向量的夹角（度）。"""
    na = math.sqrt(sum(c * c for c in a))
    nb = math.sqrt(sum(c * c for c in b))
    if na <= 0.0 or nb <= 0.0:
        return 0.0
    dot = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def blend_direction(d_current, d_target, gain: float):
    """
    以增益 gain（0~1）在当前朝向与目标方向之间插值，返回单位向量。

    闭环延迟（图像采集→处理→下发≈100~200ms）下按满增益直接指向目标会过冲：
    云台在延迟期间已多转了一个角度，于是冲过目标、目标离开视野，表现为
    "检测到→猛转→丢失"。用 gain<1 让每步只走一部分，误差按几何级数收敛。
    """
    g = max(0.0, min(1.0, gain))
    d = tuple((1.0 - g) * c + g * t for c, t in zip(d_current, d_target))
    n = math.sqrt(sum(c * c for c in d))
    if n <= 0.0:
        return tuple(d_target)
    return tuple(c / n for c in d)


def gimbal_angles_from_world_direction(d_world,
                                       yaw_sign: float = 1.0,
                                       yaw_offset_deg: float = 0.0,
                                       pitch_offset_deg: float = 0.0):
    """
    把世界系（ROS 约定：x 前 / y 左 / z 上）的目标方向转成方位角和仰角。

    返回 (yaw_deg, elevation_deg)：yaw 左正，elevation 向上为正，单位都是度。
    最外层输出适配器负责把仰角转换为具体仿真器消息所要求的 pitch 符号。
    若方向向量无效则返回 None。
    """
    x, y, z = d_world
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 0.0:
        return None
    x, y, z = x / n, y / n, z / n

    yaw_deg = math.degrees(math.atan2(y, x)) * yaw_sign + yaw_offset_deg
    theta_deg = math.degrees(math.acos(max(-1.0, min(1.0, z))))  # 与竖直向上轴的夹角
    elevation_deg = 90.0 - theta_deg                              # 目标仰角（水平=0）
    pitch_deg = elevation_deg + pitch_offset_deg
    return yaw_deg, pitch_deg
