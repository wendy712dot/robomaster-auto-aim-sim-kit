# 整合记录与验证状态

## 来源与基线

- 仿真器来源：[wendy712dot/bevy_robomaster_simulator](https://github.com/wendy712dot/bevy_robomaster_simulator)，基线分支 `fix/ros2-interface-compatibility`，提交 `7c075703d99192b940d1310bf674253f5d08d532`。
- 上游仿真器：[HNUYueLuRM/bevy_robomaster_simulator](https://github.com/HNUYueLuRM/bevy_robomaster_simulator) 与 [Blackjack200/bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)。
- 可选视觉例程：[wendy712dot/simplified-auto-aim-system](https://github.com/wendy712dot/simplified-auto-aim-system)。
- 接入经验参考：[lijjinyu-coder/armor_aim 的 feature/simulator-integration 分支](https://github.com/lijjinyu-coder/armor_aim/tree/feature/simulator-integration)。

仿真器源码与资产保持其 AGPL-3.0 许可证；可选视觉例程保留 MIT 许可证。两套视觉方案采用了不同命令接口，本仓库只把当前基线实际订阅的 `/vision_send_data` 作为默认控制接口，不把另一版本的 `/rm_gimbal/cmd` 描述为兼容接口。

## 本次整理

- 保留仿真器源码、模型/音频资产、`rm_interfaces` 与锁定依赖。
- ROS2 捕获显式使用 `Rgba8UnormSrgb`/`Depth32Float`，避免依赖已弃用的默认纹理格式 trait。
- 默认关闭 `power_rune.auto_aim_enabled` 与 `vision_test_pose`，以通用出生点和手动开启控制开始。
- 增加相对路径构建/启动脚本，默认仅构建仿真器；完整视觉例程为可选模式。
- 增加图像订阅、有限时长固定云台命令和小角度误差适配工具。
- 适配器对旧视觉结果、旧反馈、非法数值和无目标状态做保护；同一视觉结果不会在 20 Hz 定时器中重复累计修正。
- 新增接口、坐标、控制语义、停止行为和排错说明。
- 保留第一套完整 C++ 自瞄为可选例程；未迁移其测试视频和演示文件。

第二套方案仅用于吸收 QoS、TF、输入/输出职责分离、超时保护和测试记录方面的经验，没有直接迁入其 PID、GimbalCmd 或完整硬件/导航工程。

## 当前验证状态

整理环境：Ubuntu 22.04.5、ROS2 Humble、colcon、Rust/rustup 可用。

| 项目 | 状态 |
| --- | --- |
| Python 控制数学单元测试 | 通过：7/7 |
| Shell/Python 静态检查 | 通过：`bash -n` 与 `py_compile` |
| `rm_interfaces` colcon 构建 | 通过 |
| 可选 `auto_aim_ros2` C++ 构建 | 通过 |
| Rust ROS2-only release 构建 | 通过；仅有上游 prelude 未使用导入警告 |
| 仿真窗口与 Vulkan | 二进制已启动到窗口初始化；当前 Codex 沙箱无法连接 X 显示，报 `XOpenDisplayFailed`，且未安装 `vulkaninfo` |
| 图像、固定命令、闭环运行 | 未实测；必须在能访问桌面 GPU/显示会话的终端完成，不能以构建通过代替 |

## 已知限制

- 默认相机内参只适用于默认 1440x1080、45 度垂直 FOV；通用接入应读 `/camera_info`。
- `self_color=0` 是当前受控红方场景值，不是通用队伍自动识别。
- 客户端正常退出可以发送无效目标；进程崩溃时仿真端没有通用失联看门狗，需用 `F5` 关闭自动控制。
- 小角度适配器假设相机与云台近似对齐，不覆盖完整 TF、底盘姿态、弹道或大角度转换。
- 可选 C++ 例程使用固定仿真相机参数，且视频素材未包含；它是接口参考，不是所有用户必须采用的视觉算法。
- 首次 Cargo 构建下载和编译量较大，实际性能取决于 GPU、驱动、分辨率与场景配置；本仓库不承诺未经测量的最低配置或 FPS。
