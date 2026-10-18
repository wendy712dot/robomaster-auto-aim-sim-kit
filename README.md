# RoboMaster Simulator Kit

一个面向视觉组复用的 RoboMaster 仿真基础仓库。它负责提供场景、虚拟相机、云台和 ROS2 接口；你保留自己的检测、选板、PnP 与弹道代码，只需要完成“收图”和“发命令”两处适配。

```text
仿真器 -> /image_raw + /camera_info -> 你的自瞄程序
仿真器 <- /vision_send_data          <- 角度/状态适配
       -> /vision_receive_data        -> 当前云台角反馈
```

## 1. 环境

已针对以下组合整理：

- Ubuntu 22.04 桌面环境
- ROS2 Humble
- 支持 Vulkan 的 GPU 与驱动（需要可用的图形显示会话）
- Rust 1.95.0（由 `simulator/rust-toolchain.toml` 固定）

先按 [ROS2 Humble 官方 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配好软件源，再安装项目依赖：

```bash
sudo apt update
sudo apt install -y git curl build-essential cmake pkg-config clang libclang-dev \
  libssl-dev libudev-dev libasound2-dev libx11-dev libxi-dev libxrandr-dev \
  libxcursor-dev libxkbcommon-dev libwayland-dev libvulkan-dev mesa-vulkan-drivers \
  vulkan-tools python3-colcon-common-extensions \
  ros-humble-ros-base ros-humble-geometry-msgs ros-humble-sensor-msgs \
  ros-humble-tf2-msgs ros-humble-visualization-msgs ros-humble-nav-msgs \
  ros-humble-rosidl-default-generators
```

使用 [rustup 官方方式](https://rust-lang.org/tools/install/) 安装 Rust 后执行：

```bash
rustup toolchain install 1.95.0 --profile minimal
source /opt/ros/humble/setup.bash
vulkaninfo --summary
```

如果需要显示 Python 图像或构建附带的完整 C++ 视觉例程，再安装：

```bash
sudo apt install -y libopencv-dev python3-opencv ros-humble-cv-bridge \
  ros-humble-ament-index-cpp
```

GPU 驱动应按显卡型号安装；远程服务器还需要能使用 GPU/Vulkan 的图形会话，单独使用 Xvfb 并不能保证可运行。

## 2. 构建与启动

在仓库根目录执行：

```bash
bash scripts/build.sh
bash scripts/run.sh
```

首次 Rust 构建会下载并编译较多依赖。仿真器窗口出现后按 `F5` 打开自动控制；默认配置故意保持关闭，避免测试节点一启动就接管云台。

另开终端加载环境：

```bash
source scripts/env.sh
ros2 topic list
ros2 topic echo /camera_info --once
ros2 topic echo /vision_receive_data --once --qos-reliability best_effort
```

先验证两条最小链路：

```bash
# 仿真器 -> 程序：持续打印图像尺寸、编码、时间戳和接收 FPS
python3 examples/ros2_tools/image_probe.py

# 程序 -> 仿真器：发送 2 秒绝对角命令，随后主动发送停止/保持状态
python3 examples/ros2_tools/gimbal_probe.py --yaw 5 --pitch 0 --seconds 2
```

显示图像需要 OpenCV/cv_bridge：

```bash
python3 examples/ros2_tools/image_probe.py --ros-args -p show:=true
```

## 3. 接入自己的自瞄程序

推荐按以下顺序接入：

1. 订阅 `/image_raw`，按消息的 `rgb8` 编码正确转换为你的图像格式；同时订阅 `/camera_info`，不要写死内参。
2. 将图像交给原有检测、目标选择和 PnP，先只打印结果，不控制云台。
3. 订阅 `/vision_receive_data` 得到当前 `yaw/pitch`，把你的“相对角误差”转换为仿真器需要的“绝对世界角命令”。
4. 发布 `/vision_send_data`；目标无效、PnP 失败、图像或反馈超时时，明确发送 `target_state=0`、`target_distance=-1.0`。
5. 从静态、小偏角场景开始闭环，确认单位和正负方向后再增加运动目标。

`examples/ros2_tools/aim_adapter.py` 提供一个小角度、相机与云台近似对齐时的参考适配器。它订阅：

```text
/aim/error  geometry_msgs/msg/Vector3Stamped
x = 目标在相机画面中的右正 yaw 误差（度）
y = 目标在相机画面中的上正 pitch 误差（度）
z = 目标距离（米）
frame_id = camera_optical_frame
stamp = 该视觉结果对应图像的采集时间
```

默认只观察不控制。确认数据语义后启动：

```bash
python3 examples/ros2_tools/aim_adapter.py --ros-args -p enable_control:=true
```

这个示例不是通用 TF/外参/弹道解算器；相机不与云台对齐、底盘旋转或偏角较大时，应在自己的输出适配层完成坐标变换。

完整字段、坐标约定、QoS 和异常行为见 [接口说明](docs/interfaces.md)。来源、修改与当前验证结果见 [整合记录](docs/integration-notes.md)。

## 4. 可选视觉例程

仓库保留了一套简化 C++ 自瞄作为参考，不是使用仿真器的必需依赖：

```bash
bash scripts/build.sh examples
source scripts/env.sh
ros2 run auto_aim_ros2 auto_aim_open_loop
```

确认开环结果、颜色配置和方向正确后，才启用控制：

```bash
ros2 run auto_aim_ros2 auto_aim_open_loop --ros-args -p enable_control:=true
```

视频模式所引用的测试视频没有随仓库发布；如需视频模式，请自行提供视频并调整 `examples/simplified_auto_aim/config/config.yaml`。

## 5. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 收不到消息 | 两个终端是否都 source 环境、`ROS_DOMAIN_ID`、话题名、QoS |
| 图像颜色异常 | 仿真输出是 `rgb8`，OpenCV 常用 BGR |
| 云台不响应 | 窗口中是否按 `F5`、是否有其他命令发布者 |
| 越转越偏 | 度/弧度、绝对角/相对误差、yaw/pitch 正负号 |
| 振荡或延迟增长 | 队列深度、旧帧积压、命令频率、单次限幅和死区 |
| 程序退出后仍移动 | 仿真器会保留目标角；正常退出前发无效目标，强退时用 `F5` 关闭 |
| Rust 构建找不到 ROS 消息 | 先构建/source `rm_interfaces`，并确认 `libclang` 与 ROS2 环境 |
| 没有窗口或黑屏 | GPU/Vulkan 驱动、显示会话与 `DISPLAY`，不要把 Xvfb 等同于 GPU 渲染 |

## 6. 目录与许可证

```text
simulator/                     Rust + Bevy 仿真器、资产、rm_interfaces
examples/ros2_tools/           收图、固定命令、误差适配最小工具
examples/simplified_auto_aim/  可选 C++ 自瞄例程
scripts/                       环境、构建、启动脚本
docs/                          接口与整合记录
tests/                         与 ROS2 无关的控制换算单元测试
```

仿真器沿用 AGPL-3.0；`examples/simplified_auto_aim` 保留其 MIT 许可证。详见各目录中的许可证与 [整合记录](docs/integration-notes.md)。
