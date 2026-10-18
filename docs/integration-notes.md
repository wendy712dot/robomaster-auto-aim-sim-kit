# 项目说明

这份说明告诉你仓库从哪里来、做过哪些整理，以及使用时需要注意哪些限制。实际安装和接入请从根目录 README 开始。

## 来源

- 仿真器基线：[wendy712dot/bevy_robomaster_simulator](https://github.com/wendy712dot/bevy_robomaster_simulator) 的 `fix/ros2-interface-compatibility` 分支，提交 `7c075703d99192b940d1310bf674253f5d08d532`。
- 上游项目：[HNUYueLuRM/bevy_robomaster_simulator](https://github.com/HNUYueLuRM/bevy_robomaster_simulator) 和 [Blackjack200/bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)。
- 可选视觉例程：[wendy712dot/simplified-auto-aim-system](https://github.com/wendy712dot/simplified-auto-aim-system)。
- 接入经验参考：[armor_aim 的 simulator-integration 分支](https://github.com/lijjinyu-coder/armor_aim/tree/feature/simulator-integration)。

不同来源使用过不同的云台消息。本仓库只把当前仿真器实际接收的 `/vision_send_data` 作为默认控制接口。不要照搬其他版本的 `/rm_gimbal/cmd` 字段和控制参数。

## 仓库做过的整理

- 保留仿真器源码、模型、音频、ROS2 消息和锁定依赖。
- 提供从仓库根目录执行的构建、启动和环境脚本。
- 默认关闭自动控制，由你在窗口中按 `F5` 开启。
- 默认采用兼容性较高的 `auto_vsync` 显示模式，避免部分 X11/混合显卡组合无法创建显示表面。
- 提供图像接收、固定云台命令和小角度误差适配工具。
- 适配器会检查非法数值和超时；同一帧视觉结果不会重复累计修正。
- 保留一套简化 C++ 自瞄作为参考，但不要求你使用它。
- 补充图像编码、CameraInfo、角度单位、正负方向、停止行为和排查说明。

第二套方案只用于参考 QoS、TF、超时保护和输入/输出职责划分，没有把它的完整工程、PID 或另一套云台接口混入默认流程。

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

以上结果验证了“仿真器发布图像”和“测试程序发送云台命令”两条基础链路。完整视觉闭环仍需接入具体自瞄程序后，按 README 验证识别、角度转换、目标居中和目标丢失行为。

## 使用限制

- 默认内参只对应 1440x1080 和 45 度垂直视场角；你的程序应订阅 `/camera_info`。
- `self_color=0` 是当前受控红方场景值，不表示已经支持自动切换队伍。
- 仿真器会保存最后一个有效目标角。目标无效时需要主动发送停止状态；程序被强制结束后请按 `F5`。
- `aim_adapter.py` 只用于相机与云台近似对齐的小角度测试，不包含完整外参、底盘姿态、弹道或运动预测。
- 可选 C++ 例程使用固定仿真相机参数，且没有附带测试视频；它只是接入参考。
- 首次 Cargo 构建耗时和运行帧率取决于 CPU、GPU、驱动、分辨率与场景配置。

## 许可证

仿真器源码和资产沿用 AGPL-3.0。`examples/simplified_auto_aim` 保留其 MIT 许可证。继续分发或修改时请保留相应许可证和来源说明。
