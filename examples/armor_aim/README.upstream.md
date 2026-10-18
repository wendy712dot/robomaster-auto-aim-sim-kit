# auto-aiming（仿真接入版）

原 Purdue RoboMaster 自瞄工程，接入了 RoboMaster 仿真器
[bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)，
跑通"仿真图像 → 原算法 → 云台命令 → 云台转动 → 新图像"的实时闭环。

这是一个**精简过的独立仓库**：只保留仿真接入和视频模式要用的包，
MindVision 相机 SDK、雷达驱动、导航栈、Foxy 版 cv_bridge 都已移除。

## 环境

- Ubuntu 22.04 + ROS2 Humble
- OpenCV、Eigen3、libclang（编译 r2r 用）、colcon
- 仿真器：bevy_robomaster_simulator（ros2 feature），需要 Rust stable

```bash
sudo apt install -y build-essential cmake \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-camera-info-manager ros-humble-nav-msgs ros-humble-tf-transformations \
  libopencv-dev libeigen3-dev python3-opencv python3-colcon-common-extensions
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd <本仓库>
colcon build --symlink-install --packages-select \
  vision_msgs rm_interfaces opencv_armor_detector pose_estimator \
  webcam_publisher control_communicator aim_output sim_tests prm_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`src/rm_interfaces` 是仿真器的消息包（GimbalCmd 等），放在这里自瞄侧就能独立编译。
不用 `--packages-select` 全量编也行——已经精简过，只是这几个包。

## 启动

一个入口，靠参数切输入：

```bash
# 仿真模式：图像来自仿真器，内参按仿真画面，闭环输出云台命令
ros2 launch prm_launch autoaim.py input:=ros2 publish_command:=true

# 开环：只检测/解算/显示，不发命令
ros2 launch prm_launch autoaim.py input:=ros2 publish_command:=false

# 视频模式：默认跑仓库自带示例视频 src/prm_launch/video/armor_sample.avi
ros2 launch prm_launch autoaim.py input:=video
ros2 launch prm_launch autoaim.py input:=video video_path:=/path/to/your.avi
```

仿真器那边先启动（`cargo run --no-default-features --features ros2`），**启动后按 F5**
打开"自瞄订阅"开关，否则命令不会被处理。

想分别启动也行：`sim_detector.py`（仿真）、`video2detector.py`（视频）。

### 常用参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `input` | `ros2` | 输入源：`ros2` / `video` |
| `publish_command` | `true` | 是否发 `/rm_gimbal/cmd` |
| `image_width`/`image_height` | 1440/1080 | 检测尺寸，必须和仿真器 `[capture.color]` 一致 |
| `fx`/`fy`/`cx`/`cy` | 1303.675/1303.675/720/540 | 仿真内参（`input:=ros2` 时用） |
| `target_red`/`color_set` | true/red | 目标颜色，仿真里敌人是蓝的就设 false/blue |
| `control_gain`/`deadband_deg` | 0.4/1.5 | 控制增益与死区，延迟大时减小增益 |
| `max_yaw_rate_deg_s`/`max_pitch_rate_deg_s` | 60/45 | 命令速率限制，0=关 |
| `watchdog_timeout_ms` | 200 | 超过这么久没新结果就算目标丢失 |

## 模块职责

```
输入层（可切换，核心算法不感知）
  webcam_publisher/VideoCaptureNode   视频文件 → /image_raw (bgr8)
  仿真器                                → /image_raw (rgb8)

核心算法层（和真机一样，没为仿真改过逻辑）
  opencv_armor_detector   /image_raw → 缩放 → 颜色/轮廓 → /key_points
  pose_estimator          /key_points → solvePnP + yaw 优化 + 有效性过滤 → /predicted_armor

输出抽象层（只吃统一结果 AimResult）
  aim_output  /predicted_armor → AimResult{yaw_rel, pitch_rel, distance, target_valid}
              ├─ debug_output  日志 + /debug_image（候选框、角度、距离、FPS、状态）
              └─ gimbal_output /rm_gimbal/cmd（绝对角/度，BestEffort）+ 看门狗
```

数据流：

```
仿真器相机 --/image_raw--> 检测器 --/key_points--> PnP --/predicted_armor--> aim_output
     ^                                                                        |
     |                                                              /rm_gimbal/cmd
     └──────── 云台转动（仿真器 PID） ←────────────────────────────────────────┘
```

## 接口（都实测过，细节见 docs/接口确认记录.md）

| 接口 | 话题 | 说明 |
| --- | --- | --- |
| 图像 | `/image_raw` | `sensor_msgs/Image`，**rgb8**，1440×1080，约 63Hz，Reliable |
| 相机信息 | `/camera_info` | 零畸变，fx=fy=1303.675，cx=720，cy=540 |
| 检测 | `/key_points` | `vision_msgs/KeyPoints`，4 个角点 |
| 解算 | `/predicted_armor` | `vision_msgs/PredictedArmor`，x/y/z 单位 **mm** |
| 云台命令 | `/rm_gimbal/cmd` | `rm_interfaces/GimbalCmd`，**BestEffort**；`yaw` 绝对角（度，左为正）；**`pitch` 命令值 = 目标仰角**（度，±70 限幅）；`distance=-1` 表示无解 |
| 姿态 | `/tf` | 取 `odom→camera_optical_frame`；`/gimbal_pose` 等 pose 字段是空的，别用 |

内部 `AimResult`：`yaw_rel_deg` 相对光轴方位角（右正）、`pitch_rel_deg` 相对水平仰角（上正）、
`distance_m` 米、`target_valid` 是否有效。

下发角度算法（每帧用最新 TF 重算，误差不累积）：

```
d_world   = R(world←camera_optical_frame) · d_cam
yaw_cmd   = atan2(d_world.y, d_world.x)
pitch_cmd = 90 - acos(d_world.z)      # 即目标仰角
```

## 目标丢失行为

| 情况 | 上游 | 本工程 |
| --- | --- | --- |
| 无装甲 / PnP 无效 | 发全 0 `predicted_armor` | 发 `distance=-1`，仿真器停追踪、云台保持姿态 |
| 上游静默（IDLING） | 不发消息 | 看门狗超时 → 发 `distance=-1` |
| 图像中断 | 无新消息 | 同上 |
| 取不到 TF | — | 视为无效，只输出调试信息，不发命令 |

## 文档

`docs/` 里有：接口确认记录、部署与排查记录、测试记录、任务完成情况对照报告、
系统数据流图、演示材料清单。演示视频在 `docs/demo/`。

## 说明

`src/cv_bridge` 之类没有——用的是系统 `ros-humble-cv-bridge`。
MindVision 相机（`mv_publisher`）和雷达、导航相关包已移除，真机用的话需要从原仓库取回。
