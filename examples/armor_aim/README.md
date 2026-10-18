# armor_aim 多节点视觉例程

这是第二套参考视觉程序的仿真精简版，来源为
[`lijjinyu-coder/armor_aim` 的 `feature/simulator-integration` 分支](https://github.com/lijjinyu-coder/armor_aim/tree/feature/simulator-integration)，
基线提交为 `d445903`。原工程采用 GPL-3.0，许可证见本目录 `LICENSE`。

本目录保留它的多节点主链路：

```text
/image_raw → opencv_armor_detector → /key_points
           → pose_estimator → /predicted_armor
           → aim_output → /vision_send_data
```

检测器、PnP 和基于 TF 的世界方向换算来自原方案。为了与本仓库选定的仿真器一致，最外层输出由原来的 `/rm_gimbal/cmd` 适配为 `/vision_send_data`。当前仿真器的 TF 光学坐标与 `VisionSendData` 绝对角基准存在固定轴偏置，因此适配器从 TF 取相对修正量，再锚定到 `/vision_receive_data` 的真实 yaw/pitch；不能把 TF 绝对角直接写入控制消息。无目标时发送 `target_state=0` 和 `target_distance=-1.0`。

## 构建

从仓库根目录执行：

```bash
bash scripts/build.sh armor
source scripts/env.sh
```

## 开环识别

仿真器保持运行，在新终端执行：

```bash
source scripts/env.sh
ros2 launch prm_launch sim_detector.py publish_command:=false show_window:=true
```

先确认终端持续输出检测、PnP 和 `AimResult`，调试窗口中能看到装甲板结果。默认检测当前场景中的蓝色敌方装甲板。仿真启动文件已经使用本机实测稳定的暗场阈值、全图搜索和 5 帧丢失容忍；正常时日志应持续为 `TRACKING` 或 `FIRE`、`valid=1`，而不是长期 `NO_ARMOR`。

## 闭环追踪

先让仿真器保持 `auto-aim=OFF camera=ROBOT`，并确保没有其他程序发布 `/vision_send_data`，再执行：

```bash
ros2 launch prm_launch sim_detector.py \
  publish_command:=true \
  allow_fire:=false \
  show_window:=true
```

启动文件默认采用实测通过的 `control_gain=0.1`、yaw 10°/s、pitch 8°/s。目标应逐渐回到中心；若越转越偏、突然转向天空或剧烈振荡，立即按 `F5` 关闭控制或按 `Ctrl+C`，不要先提高参数。

例程即使显示 `status: FIRE`，默认参数 `allow_fire:=false` 也只会发送 `target_state=1`，不会请求发射。只有专门测试发射闭环时才显式设置 `allow_fire:=true`。

### 打开 F5 前的安全检查

保持 `auto-aim=OFF`，在两个终端分别读取当前反馈和一条命令：

```bash
ros2 topic echo /vision_receive_data --once
ros2 topic echo /vision_send_data --once
```

有效命令应为 `target_state: 1`、`target_distance > 0`，命令 yaw/pitch 与当前反馈只相差一个小修正量。实机验证时的示例是 yaw 从 `-66.80°` 修正为 `-65.52°`、pitch 从 `30.59°` 修正为 `30.23°`。如果静止场景下一条命令突然相差几十度甚至接近 180°，不要开启 F5；这通常表示错误地把 TF 绝对角直接写进了 `VisionSendData`。

检查通过后，点击仿真器窗口并按 `F5`，确认左下角变为 `auto-aim=ON`。这时才开始真正驱动云台。

### 验收标准

1. 开环连续输出 `valid=1`，距离和角度稳定，实测处理约 45–60 FPS。
2. 打开 F5 后目标逐渐回到中心，yaw/pitch 相对误差总体减小。
3. 移动己方或目标后能重新追踪；目标丢失时发送无效状态，重新出现后恢复。
4. 关闭并重启仿真器与例程后仍能复现。

原方案的完整说明保存在 `README.upstream.md`，其中 `/rm_gimbal/cmd`、旧仿真器 PID 和 pitch 标定不适用于本仓库，接入时以本文件和仓库根目录的接口说明为准。
