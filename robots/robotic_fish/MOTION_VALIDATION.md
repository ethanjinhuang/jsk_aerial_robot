# 单舵机运动调整验证记录

## 2026-11-15 ADC 与水听器方向重映射

周期转向及低信号检查改用 ADC0=左、ADC1=右、ADC2=前。
Share 的原始瞬时值、原始周期均值和校准周期均值统一按前／左／右显示，
对应 ADC2／ADC0／ADC1。物理 ADC 编号、采样顺序和校准表保持原样；
运动锁存配置 schema 5 记录方向布局，离线回放须显式选择新旧接线。

运动包 91 项、声学包 191 项单元测试及 5 项离线 bag 测试通过；
两包独立构建安装、安装后导入和入口解析通过。1.5／2 Hz 两种初始符号的隔离
周期 ROS 回归及实际录包回读通过，未连接真实硬件。保留用户工作区现有摆幅 256，
个人摆幅配置不纳入通道映射提交；launch 测试验证继承 YAML 中的摆幅。
完整验证、适用边界和日志见[声学包验证记录](../robotic_fish_sonic/VALIDATION.md)。

## 2026-11-15 Option 切换跟随／远离声源

当前单舵机声学控制原未使用 `buttons[9]`。现默认将 Option 作为按下沿触发的
方向符号切换键，按当前实机定义 −1 为 FOLLOW（跟随）、+1 为 AVOID（远离）。
长按只切换一次；待生效期间连续新按基于待生效符号翻转，保留待生效的阈值、
偏置和补偿。复用在线请求路径，在下一有效完整周期生效，不重置相位或解除停止。
可通过 `polarity_toggle_button` 重映射；有效配置和事件来源同步保存。
Share 增加当前 Sound mode，符号不同的待生效请求另显示 Pending mode。

- 运动包 90 项、声学包 185 项单元测试通过，共 275 项。
- 新增回归覆盖长按、同周期连续切换、保留待生效调参、缺失／无效 Joy 不重置按下沿、
  Cross／冲突模式键优先、停止状态仅预选、禁用声学、关机及按键映射冲突。
- 两包在 `/tmp/fish-option-toggle` 独立构建安装通过。
- 安装版本在 1.5／2 Hz、两种初始符号下通过隔离 ROS 测试；Option 长按跨越完整周期
  不重复，松开再按反向切换，Share、相位、既有故障处理正常。
- 实际录包回读确认两次按键请求的符号和 `source=joystick_option`，并验证周期、配置及事件关联。

测试仅使用隔离 master、合成 ADC/Joy，不连接真实硬件。
日志：`/tmp/fish-option-motion-unit.log`、`/tmp/fish-option-sonic-unit.log`、
`/tmp/fish-option-build.log`、`/tmp/fish-option-cycle-ros.log`。

## 2026-11-15 方向符号在线调参

`left_bias_sign` 现可与阈值、偏置和电压补偿一起通过
`/sonic_teleop_node/set_acoustic_steering` 暂存，在下一有效完整周期原子生效。
`-1`／`+1` 指定方向；服务中的 `0` 保持当前生效符号。不会重置摆尾相位、
立即修改当前偏置或解除停止锁存。补偿越界时符号也继续等待，非法符号不覆盖已有请求。
响应包含当前／待生效符号，配置快照、事件消息和应用日志同步记录。

- 全部单元测试：运动包 84 项、声学包 184 项，共 268 项通过。
- 两包在 `/tmp/fish-online-sign` 独立配置、构建和安装通过。
- 安装后的真实 ROS 服务在 1.5／2 Hz、两种初始符号下通过在线反转与恢复、
  周期对齐、无效符号拒绝、补偿等待、停止恢复和 ADC／Joy 异常路径。
- 实际 `record.launch` 生成的 bag 回读通过；接受／生效事件的符号一致，
  配置和状态正确记录符号改变。升级后的三字段 YAML 调用仍通过默认 0 保留当前符号。

本次测试使用本机隔离 master、合成 ADC/Joy，不连接真实硬件。
`SetAcousticSteering` 和 `AcousticSteeringEvent` 的 ROS MD5 已改变，两端需更新、
重新编译两包、source 并重启节点；之后可在线切换，无需每次重启。
日志：`/tmp/fish-online-sign-motion-unit-final.log`、`/tmp/fish-online-sign-sonic-unit-final.log`、
`/tmp/fish-online-sign-build.log`、`/tmp/fish-online-sign-motion-build.log`、
`/tmp/fish-online-sign-cycle-ros.log`。原有历史验证如下。

验证日期：2026-10-20。

本次将 `sonic_teleop.py` 的轨迹计算和手柄状态机提取到
`src/robotic_fish/sonic_motion.py`，增加停止锁存、有效 Joy 接收超时和
显式恢复。手动位置与自动摆尾中心均直接采用摇杆输入，不设置偏置变化
速率限制。下文保留声学功能接入前的基础运动验证；新增的周期趋声功能见
[周期趋声验证记录](../robotic_fish_sonic/docs/cycle_steering_validation.md)。

## 基础运动验证（声学功能接入前）

| 检查 | 结果 |
| --- | --- |
| 运动与 ROS 适配层单元测试 | 26 项通过，0 错误、0 失败 |
| `robotic_fish_sonic` 回归测试 | 初版重构时 91 项通过；本次未修改声学包 |
| 原版正弦轨迹对照 | 392 个样本一致，浮点误差小于 `1e-10` |
| 构建、测试和安装 | 在 `/tmp` 下独立构建／安装空间完成 |
| 安装后的 Python 模块与节点 | 可导入；与源文件内容一致 |
| launch 配置加载 | 默认参数和覆盖参数两种情况通过 |
| 安装后的节点通信测试 | 独立 ROS master、合成 Joy 和测试话题下通过 |
| 其他运动程序 | `teleop.py`、`advance_teleop.py`、`spring_teleop.py` 与修改前一致 |

通信测试未启动桥、ADC、DAC 或执行器。测试 master 使用本机端口 11428；
输入、输出分别重映射到 `/motion_test/joy` 和 `/motion_test/targets`。
启用 `/use_sim_time` 且不发布 `/clock`，确认墙上时间控制循环仍正常运行：

- 自动摆尾覆盖正负峰值，输出间隔符合约 50 Hz，目标处于软件范围内。
- 停止后持续回中；保持自动键不会恢复。
- 释放按键并回中后，新按手动键可恢复；位置目标直接跟随摇杆输入。
- 自动模式中改变方向输入，中心偏置立即采用新值，摆尾相位继续推进；
  到达位置边界时摆幅缩小至零，回到较小偏置时恢复周期摆动。
- 短 Joy 消息和有效消息中断均停止运动；连接恢复不会自行恢复运动。
- 显式释放并重新按自动键可恢复；节点正常退出。

独立构建目录为 `/tmp/robotic-fish-motion-check`，其中保留构建、测试、
安装和 launch 检查日志；取消限速后的构建和安装日志使用 `direct-` 前缀。
最新安装空间通信测试日志位于 `/tmp/fish-motion-ros-w_mh0j5l`。
这些临时目录仅是本次本机验证证据，不是仓库运行依赖。
本机旧版 nose 在 Python 3.10 下的兼容处理也仅位于 `/tmp`。

## 复现

先在 ROS 工作空间编译 `robotic_fish` 并 source 对应的 setup 文件，
再从仓库根目录执行：

```bash
python3 -m unittest discover -s robots/robotic_fish/test -v
python3 robots/robotic_fish/test/sonic_ros_smoke.py
```

安装空间验证可为通信测试增加 `--node-script`，指向安装后的
`lib/robotic_fish/sonic_teleop.py`。通信测试使用的端口需空闲。

## 尚待现场确认

本次未连接真实舵机、手柄或机器鱼，未执行水中实验，也未运行远端 CI。
软件限位、左右转向符号和摆频／摆幅仍需现场确认。
Joy 看门狗验证消息接收，不验证无线链路状态；真实手柄断连时驱动是否
继续重复旧消息，以及串口失联时执行器的行为，需要单独实测。
新增声信号到中心偏置的映射已经完成软件验证，尚未验证水中趋声效果。
