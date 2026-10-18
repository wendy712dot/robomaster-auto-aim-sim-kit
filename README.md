# RoboMaster Simulator Kit

这个仓库用于调试你自己的视觉自瞄程序。仿真器生成机器人相机画面；你的程序读取画面、识别目标，再把云台角度命令发回仿真器。

你不需要学习 Rust 或 Bevy，也不需要采用仓库里的视觉算法。接入时只需要完成两件事：

1. 从 ROS2 读取图像和相机参数。
2. 把自瞄结果转换为仿真器的云台命令。

```text
仿真器 --图像和相机参数--> 你的自瞄程序
仿真器 <--云台角度命令---- 你的自瞄程序
```

## 1. 先认识几个词

- **ROS2**：程序之间传递数据的通信系统。
- **节点（node）**：一个正在运行并参与 ROS2 通信的程序。
- **话题（topic）**：节点传递某一类数据的通道，例如 `/image_raw` 传图像。
- **消息（message）**：话题中每次传递的数据及其字段。
- **QoS**：消息传输方式。两端设置不兼容时，即使话题名正确也可能收不到数据。

后文的命令都在仓库根目录执行。出现“另开终端”时，请打开一个新的终端窗口，并再次进入仓库目录。

## 2. 准备环境

建议使用：

- Ubuntu 22.04 桌面版
- ROS2 Humble
- 支持 Vulkan 的显卡和正确安装的驱动
- 可正常打开图形窗口的桌面会话

先按照 [ROS2 Humble 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置 ROS2 软件源，然后安装基础依赖：

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

按照 [Rust 官方说明](https://rust-lang.org/tools/install/) 安装 rustup，再安装本项目固定使用的 Rust 版本：

```bash
rustup toolchain install 1.95.0 --profile minimal
source /opt/ros/humble/setup.bash
vulkaninfo --summary
```

最后一条命令应列出显卡和 Vulkan 信息。若报错，请先解决显卡驱动问题。

只有在显示图像或构建附带的 C++ 自瞄例程时，才需要额外安装：

```bash
sudo apt install -y libopencv-dev python3-opencv ros-humble-cv-bridge \
  ros-humble-ament-index-cpp
```

## 3. 构建并启动仿真器

第一次构建会下载和编译较多 Rust 依赖，耗时较长属于正常现象。

在终端 A 执行：

```bash
cd /你的路径/robomaster-sim-kit
bash scripts/build.sh
bash scripts/run.sh
```

成功后会出现仿真器窗口。先点击窗口，再按一次 `F5` 打开自动控制；再次按下会关闭。默认关闭是为了防止测试程序意外接管云台。左下角必须显示 `auto-aim=ON`；ThinkPad 等笔记本若把 F5 用作硬件功能键，请使用 `Fn+F5`。

左下角还会显示相机模式。闭环自瞄时必须使用 `camera=ROBOT`，此时 `/image_raw` 是随云台转动的第一人称画面。`F3` 会在 `ROBOT`、`THIRD` 和 `FREE` 三种模式间循环；后两种适合观察场景，但画面不会随受控云台转动，不能用来判断闭环是否居中。

不要关闭终端 A。另开终端 B，进入同一个仓库并加载 ROS2 环境：

```bash
cd /你的路径/robomaster-sim-kit
source scripts/env.sh
ros2 topic list
```

列表中应能看到 `/image_raw`、`/camera_info`、`/vision_receive_data` 和 `/vision_send_data`。

## 4. 先验证图像和云台

### 4.1 验证接收图像

在终端 B 执行：

```bash
python3 examples/ros2_tools/image_probe.py
```

正常时会持续打印图像尺寸、`rgb8` 编码、时间戳和接收帧率。这证明“仿真器到你的程序”已经连通。

需要显示画面时使用：

```bash
python3 examples/ros2_tools/image_probe.py --ros-args -p show:=true
```

按 `Ctrl+C` 停止测试。

### 4.2 验证控制云台

确认仿真器窗口中自动控制已经打开，然后执行：

```bash
python3 examples/ros2_tools/gimbal_probe.py --yaw 5 --pitch 0 --seconds 2
```

云台应向 5 度的绝对 yaw 方向运动。程序运行 2 秒后会主动发送停止追踪状态。测试时只运行一个云台命令程序，避免多个程序互相覆盖。

如果方向与预期不一致，先不要接入视觉算法，按照[接口说明](docs/interfaces.md)检查单位、正负方向和控制语义。

## 5. 接入你自己的程序

按下面的顺序做，出现问题时容易判断是哪一段造成的。

1. 订阅 `/image_raw`，把 `rgb8` 图像转换为你的程序使用的格式。OpenCV 通常使用 BGR。
2. 订阅 `/camera_info`，使用其中的相机内参。不要直接沿用旧视频的内参。
3. 先让原有检测、选板和 PnP 处理实时画面，但暂时不要控制云台。
4. 订阅 `/vision_receive_data`，读取当前云台 `yaw` 和 `pitch`。
5. 把你的相对角度误差或三维目标位置转换为绝对角命令。
6. 向 `/vision_send_data` 发布命令，最后从静止目标和小角度偏差开始闭环测试。

目标丢失、PnP 失败、图像中断或反馈超时时，必须明确发送：

```text
target_state = 0
target_distance = -1.0
```

仿真器会保留最后一个有效目标角，所以只停止发布消息并不能保证云台停止追踪。

## 6. 可选的小角度适配器

如果你的程序能输出目标相对于画面中心的角度误差，可以先用 `aim_adapter.py` 验证最小闭环。它接收：

```text
话题：/aim/error
类型：geometry_msgs/msg/Vector3Stamped
x：目标在画面右侧为正的 yaw 误差，单位度
y：目标在画面上方为正的 pitch 误差，单位度
z：目标距离，单位米
frame_id：camera_optical_frame
stamp：产生该结果的原始图像时间戳
```

适配器默认不控制云台。确认你的字段符合上面的定义后运行：

```bash
python3 examples/ros2_tools/aim_adapter.py --ros-args -p enable_control:=true
```

它会限制单次修正幅度，并在视觉结果或反馈过期时发送无效目标。同一帧结果只计算一次修正。

这个适配器只适用于相机与云台近似对齐、小角度误差的入门测试。相机存在明显外参、底盘旋转或偏角较大时，请在自己的程序中通过 TF/外参完成坐标转换。

## 7. 可选 C++ 自瞄例程

仓库附带了一套简化 C++ 自瞄，用来参考如何连接 ROS2；使用仿真器并不依赖它。

```bash
bash scripts/build.sh examples
source scripts/env.sh
ros2 run auto_aim_ros2 auto_aim_open_loop
```

先检查识别框和角度输出。窗口中的 `Target: TRACKING` 表示目标已经通过连续帧确认；`DETECTED` 只是刚检测到，`TEMP_LOST` 表示暂时丢失。只有 `TRACKING` 且 PnP 成功时，例程才发送有效追踪命令。

确认颜色、相机参数、距离和方向均正确后，切换仿真器为 `auto-aim=ON camera=ROBOT`，再从每次最多修正 0.5 度开始闭环：

```bash
ros2 run auto_aim_ros2 auto_aim_open_loop --ros-args \
  -p enable_control:=true \
  -p max_correction_deg:=0.5
```

`Control: ON` 只说明例程允许发布控制，不代表仿真器已经打开接收。目标应逐渐回到画面中心，Yaw/Pitch 误差总体减小。验证稳定后可把单次修正提高到 `1.0`；第一次测试不建议直接使用默认的 `2.0`。

如果显示 `Control: ON` 但云台不动，在另一个已执行 `source scripts/env.sh` 的终端检查：

```bash
ros2 topic info /vision_send_data --verbose
ros2 topic echo /vision_send_data --once
```

正常时应各有一个发布者和订阅者；单条消息应有 `target_state: 1`、大于零的 `target_distance`，以及有限的 `yaw/pitch`。有订阅者并不等于 F5 已打开，最终仍以仿真器左下角的 `auto-aim=ON` 为准。

启动后可能需要几帧才能从 `DETECTED` 进入 `TRACKING`，也要等待第一条云台反馈。若数秒后才开始运动，但随后能稳定追踪，这是正常的初始化过程。

仓库没有附带上游测试视频；视频模式需要你自行提供视频并修改 `examples/simplified_auto_aim/config/config.yaml`。

## 8. 常见问题

| 现象 | 先检查什么 |
| --- | --- |
| 找不到 ROS2 或消息类型 | 是否执行了 `source scripts/env.sh`，ROS2 Humble 是否安装完整 |
| 看不到预期话题 | 仿真器是否仍在运行，两个终端的 `ROS_DOMAIN_ID` 是否相同 |
| 收不到图像 | 话题名和 QoS；先运行仓库自带的 `image_probe.py` |
| 图像颜色异常 | 仿真器输出 `rgb8`，OpenCV 常用 BGR |
| `Control: ON` 但云台不响应 | 仿真器是否为 `auto-aim=ON`；`Target` 是否为 `TRACKING`；命令是否为 `target_state: 1`；是否还有其他发布者 |
| 云台在动但图像不随动 | 按 `F3` 切到 `camera=ROBOT`；`THIRD/FREE` 观察视角不绑定云台 |
| 云台越转越偏 | 度/弧度、绝对角/相对误差、yaw/pitch 正负号 |
| 控制振荡或延迟增长 | 是否积压旧帧、单次命令是否过大、是否重复使用旧结果 |
| 程序退出后仍移动 | 正常退出前发送无效目标；强制退出后在窗口按 `F5` |
| 没有窗口或黑屏 | 显卡驱动、Vulkan、桌面显示会话和 `DISPLAY` |
| 报 `Invalid surface` | 保持 `present_mode = "auto_vsync"`；运行 `vulkaninfo --summary` 和 `vkcube`；混合显卡再尝试指定适配器 |

如果机器同时有 Intel 核显和 NVIDIA 独显，先运行 `prime-select query` 查看 PRIME 模式。若强制 `nvidia` 模式无法正常创建 Vulkan 窗口，可切回按需模式并重启：

```bash
sudo prime-select on-demand
sudo reboot
```

重启后先确认 `vulkaninfo --summary` 和 `vkcube` 能正常运行。若 `vkcube` 正常但仿真器仍报 `Invalid surface`，再根据设备名称指定能够驱动当前桌面的显卡：

```bash
WGPU_BACKEND=vulkan WGPU_ADAPTER_NAME=intel bash scripts/run.sh
```

如果桌面由 NVIDIA 驱动，把 `intel` 换成设备名称中能够唯一匹配 NVIDIA 显卡的部分。`WGPU_ADAPTER_NAME` 不区分大小写。

完整字段和坐标约定见[接口说明](docs/interfaces.md)。来源、许可证及验证范围见[项目说明](docs/integration-notes.md)。

## 9. 目录和许可证

```text
simulator/                      仿真器源码、资产和 ROS2 消息
examples/ros2_tools/            收图、固定命令和误差适配工具
examples/simplified_auto_aim/   可选 C++ 自瞄例程
scripts/                        构建、启动和环境脚本
docs/                           接口与项目说明
tests/                          控制换算单元测试
```

仿真器沿用 AGPL-3.0；`examples/simplified_auto_aim` 保留其 MIT 许可证。
