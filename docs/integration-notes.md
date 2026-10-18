# 项目说明

这份说明告诉你仓库从哪里来、做过哪些整理，以及使用时需要注意哪些限制。实际安装和接入请从根目录 README 开始。

## 来源

- 仿真器基线：[wendy712dot/bevy_robomaster_simulator](https://github.com/wendy712dot/bevy_robomaster_simulator) 的 `fix/ros2-interface-compatibility` 分支，提交 `7c075703d99192b940d1310bf674253f5d08d532`。
- 上游项目：[HNUYueLuRM/bevy_robomaster_simulator](https://github.com/HNUYueLuRM/bevy_robomaster_simulator) 和 [Blackjack200/bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)。
- 可选视觉例程：[wendy712dot/simplified-auto-aim-system](https://github.com/wendy712dot/simplified-auto-aim-system)。
- 多节点视觉例程：[lijjinyu-coder/armor_aim](https://github.com/lijjinyu-coder/armor_aim/tree/feature/simulator-integration)，基线提交 `d445903`。

不同来源使用过不同的云台消息。本仓库只把当前仿真器实际接收的 `/vision_send_data` 作为默认控制接口。不要照搬其他版本的 `/rm_gimbal/cmd` 字段和控制参数。

## 仓库做过的整理

- 保留仿真器源码、模型、音频、ROS2 消息和锁定依赖。
- 提供从仓库根目录执行的构建、启动和环境脚本。
- 默认关闭自动控制，由你在窗口中按 `F5` 开启。
- 默认采用兼容性较高的 `auto_vsync` 显示模式，避免部分 X11/混合显卡组合无法创建显示表面。
- 提供图像接收、固定云台命令和小角度误差适配工具。
- 适配器会检查非法数值和超时；同一帧视觉结果不会重复累计修正。
- 保留单进程简化 C++ 自瞄和多节点 armor_aim 两套例程，但不要求你使用其中任何一套。
- 补充图像编码、CameraInfo、角度单位、正负方向、停止行为和排查说明。

armor_aim 只保留仿真闭环需要的检测、PnP、输出和启动包；硬件相机、导航与旧云台接口未带入。检测与 PnP 保持原方案结构，最外层输出改为本仓库统一的 `VisionSendData`。

## 已完成的检查

整理环境为 Ubuntu 22.04.5、ROS2 Humble。

| 检查项 | 结果 |
| --- | --- |
| 控制换算单元测试 | 7 项全部通过 |
| Shell 和 Python 语法检查 | 通过 |
| `rm_interfaces` ROS2 消息构建 | 通过 |
| 可选 `auto_aim_ros2` C++ 构建 | 通过 |
| Rust ROS2-only release 构建 | 通过 |
| 仿真器图形窗口 | 通过；ThinkPad 双显卡切换为 PRIME `on-demand` 后可正常显示 |
| ROS2 图像接收与显示 | 通过；1440x1080、`rgb8`，实测约 21-22 FPS |
| 固定云台命令 | 通过；`gimbal_probe.py` 可控制云台并在结束后发送停止状态 |
| C++ 例程开环识别 | 通过；可检测灯条和装甲板并输出 PnP 角度、距离 |
| C++ 例程闭环追踪 | 通过；低速限幅下可居中、随动，目标丢失后可恢复 |
| armor_aim ROS2 包构建 | 通过；6 个包完成构建 |
| armor_aim 纯控制测试 | 22 项全部通过，包含 TF/反馈基准对齐和 ±180° 环绕 |
| armor_aim 开环识别 | 通过；暗场参数下稳定识别，实测处理约 45–60 FPS |
| armor_aim 闭环追踪 | 通过；TF 相对修正锚定反馈角后可低速居中和随动，默认禁止开火 |
| 完整重启复现 | 通过；重新启动仿真器和例程后闭环仍可工作 |

以上结果验证了仿真器与仓库附带两套视觉例程之间的最小闭环。你自己的视觉程序仍需按 README 独立验证识别、角度转换、目标居中和目标丢失行为。

实机排查还确认了两点：视觉窗口的 `Control: ON` 只代表例程允许发布，仿真器仍需单独切换为 `auto-aim=ON`；闭环图像必须使用随云台运动的 `camera=ROBOT`，第三人称和自由相机只适合观察。

armor_aim 初次直接把 TF 世界绝对角发布到 `VisionSendData` 时，命令与反馈相差接近 180°，开启控制后云台转向天空。当前适配器只从 TF 取相对方向修正，并以 `/vision_receive_data` 为绝对角基准；实测修复后一条命令与反馈分别只相差约 1.28° yaw 和 0.37° pitch，随后闭环通过。检测器默认阈值也按本机暗场画面调整为 HSV `hue=40`、`saturation>=60`、`value>=80`，关闭局部搜索并容忍 5 帧短暂丢失。

## 使用限制

- 默认内参只对应 1440x1080 和 45 度垂直视场角；你的程序应订阅 `/camera_info`。
- `self_color=0` 是当前受控红方场景值，不表示已经支持自动切换队伍。
- 仿真器会保存最后一个有效目标角。目标无效时需要主动发送停止状态；程序被强制结束后请按 `F5`。
- `aim_adapter.py` 只用于相机与云台近似对齐的小角度测试，不包含完整外参、底盘姿态、弹道或运动预测。
- 可选 C++ 例程使用固定仿真相机参数，且没有附带测试视频；它只是接入参考。
- 首次 Cargo 构建耗时和运行帧率取决于 CPU、GPU、驱动、分辨率与场景配置。

## 许可证

仿真器源码和资产沿用 AGPL-3.0。`examples/simplified_auto_aim` 保留 MIT 许可证，`examples/armor_aim` 保留 GPL-3.0 许可证。继续分发或修改时请保留相应许可证和来源说明。
